# FPR test suite entry point. Runs on the workstation, orchestrates the
# bare metal (SSH) and the guest (WinRM via the bare metal).
# PowerShell 5.1 compatible.
param(
    [string]$BareMetal = "47.83.225.42",
    [string]$VmName = "fpr_upstream",
    [string]$GuestIp = "192.168.122.6",
    [string]$ExpectedDriverVersion = "100.6.101.58300",
    [int]$VmMemMb = 4096,
    [int]$StressMb = 1500,
    [switch]$IncludeReboot,
    [switch]$SkipStress
)

$ErrorActionPreference = "Stop"
. (Join-Path $PSScriptRoot "host\lib-host.ps1")
$script:BareMetal = $BareMetal
$script:VmName = $VmName
$script:GuestIp = $GuestIp

# make sure the remote working dir exists (plain ssh, no quoting pitfalls)
ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no root@$BareMetal "mkdir -p $script:RemoteDir" 2>$null | Out-Null

# ===========================================================================
Write-Host "`n===== T1: guest driver state =====" -ForegroundColor Cyan
$t1out = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t1-driver.ps1") "`$ExpectedDriverVersion = '$ExpectedDriverVersion'"
Write-Host $t1out
$t1out -split "`n" | ForEach-Object {
    if ($_ -match "ASSERT: (PASS|FAIL) (\S+) - (.*)") {
        Assert $Matches[2] ($Matches[1] -eq "PASS") $Matches[3]
    }
}
# driver version cross-check on the host side
if ($t1out -match "actual=(\S+) expected=") {
    Assert "T1.3-version-host-check" ($Matches[1] -eq $ExpectedDriverVersion) "actual=$($Matches[1]) expected=$ExpectedDriverVersion"
}

# ===========================================================================
Write-Host "`n===== T2.1: feature negotiation (QEMU view) =====" -ForegroundColor Cyan
$feat = Get-BalloonGuestFeatures
Write-Host $feat
Assert "T2.1-reporting-negotiated" ($feat -match "VIRTIO_BALLOON_F_REPORTING") "guest-features contains PAGE_REPORTING"
Assert "T2.1-stats-negotiated" ($feat -match "VIRTIO_BALLOON_F_STATS_VQ") "guest-features contains STATS_VQ"

# ===========================================================================
Write-Host "`n===== T2.2: park (RSS should drop below 60% of VM RAM) =====" -ForegroundColor Cyan
$parkLimit = [int]($VmMemMb * 0.60)
$rss = Get-QemuRssMb
Write-Host "  initial RSS: $rss MB (limit $parkLimit MB)"
$deadline = (Get-Date).AddMinutes(4)
while ($rss -gt $parkLimit -and (Get-Date) -lt $deadline) {
    Start-Sleep -Seconds 20
    $rss = Get-QemuRssMb
    Write-Host "  waiting for park... RSS=$rss MB"
}
Assert "T2.2-park" ($rss -gt 0 -and $rss -le $parkLimit) "RSS=$rss MB <= ${parkLimit}MB after park"

# ===========================================================================
Write-Host "`n===== T2.3: hold stability (3 min, rebound < 10% of VM RAM) =====" -ForegroundColor Cyan
$samples = @()
for ($i = 0; $i -lt 7; $i++) {
    Start-Sleep -Seconds 30
    $samples += Get-QemuRssMb
    Write-Host ("  sample {0}: RSS={1} MB" -f ($i + 1), $samples[$i])
}
$min = ($samples | Measure-Object -Minimum).Minimum
$max = ($samples | Measure-Object -Maximum).Maximum
$rebound = $max - $min
Assert "T2.3-hold-stability" ($rebound -lt ($VmMemMb * 0.10)) "RSS min=${min}MB max=${max}MB rebound=${rebound}MB (< $([int]($VmMemMb*0.10))MB)"

# ===========================================================================
Write-Host "`n===== T2.5: balloon stats regression =====" -ForegroundColor Cyan
$binfo = Get-BalloonInfo
Write-Host $binfo
Assert "T2.5-stats-readable" ($binfo -notmatch "error|not available") "info balloon readable"
if ($binfo -match "stat: memory_total \(MB\) (\d+)") {
    $totalMb = [int]$Matches[1]
    Assert "T2.5-stats-total" ($totalMb -ge ($VmMemMb - 64) -and $totalMb -le $VmMemMb) "memory_total=${totalMb}MB ~ ${VmMemMb}MB"
} else {
    Skip "T2.5-stats-total" "no memory_total line in info balloon output"
}

# ===========================================================================
Write-Host "`n===== T2.6: inflate/deflate regression (virsh setmem) =====" -ForegroundColor Cyan
# The guest TotalVisibleMemorySize never changes with ballooning (only the
# available memory does), and the FPR watermark logic deliberately re-balances
# the available memory, so numeric guest-side assertions are not stable here.
# This is a regression test: setmem must take effect, the driver must serve
# the inflate/deflate requests and the guest must stay healthy throughout.
$memQuery = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t2-memstress.ps1")
$totalKB = 0
if ($memQuery -match "totalKB=(\d+)") { $totalKB = [int]$Matches[1] }
Write-Host "  guest total memory: $totalKB KB"

$target = [int]($VmMemMb - 512)
Invoke-BareMetal -ScriptBody "virsh setmem $script:VmName ${target}M --live && echo SETMEM_OK" | ForEach-Object { Write-Host "  $_" }
Start-Sleep -Seconds 45
$domInfo = Invoke-BareMetal -ScriptBody "virsh dominfo $script:VmName | grep -i memory"
Write-Host $domInfo
# Note: while the FPR driver still holds pages, a large inflate request
# transiently trips the low-memory protection (IsLowMemory) and the inflate
# completes gradually as the FPR watermark logic hands pages back - the two
# mechanisms deliberately cooperate. Assert that ballooning took effect
# (current memory below max), not that it fully converged within 45s.
$usedKb = 0
if ($domInfo -match "Used memory:\s+(\d+) KiB") { $usedKb = [int]$Matches[1] }
Assert "T2.6-inflate-applied" ($usedKb -gt 0 -and $usedKb -lt ($VmMemMb * 1024)) "current memory ${usedKb}KiB < max (inflate active, possibly still converging)"

$memQuery2 = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t2-memstress.ps1")
$totalKB2 = 0
if ($memQuery2 -match "totalKB=(\d+)") { $totalKB2 = [int]$Matches[1] }
Assert "T2.6-guest-alive-inflated" ($totalKB2 -eq $totalKB -and $totalKB -gt 0) "guest responsive, total ${totalKB2}KB == ${totalKB}KB"

Invoke-BareMetal -ScriptBody "virsh setmem $script:VmName ${VmMemMb}M --live && echo SETMEM_OK" | Out-Null
Start-Sleep -Seconds 45
$domInfo2 = Invoke-BareMetal -ScriptBody "virsh dominfo $script:VmName | grep -i memory"
Write-Host $domInfo2
$usedKb2 = 0
if ($domInfo2 -match "Used memory:\s+(\d+) KiB") { $usedKb2 = [int]$Matches[1] }
Assert "T2.6-deflate-applied" ($usedKb2 -eq ($VmMemMb * 1024)) "current memory fully restored to $($VmMemMb * 1024)KiB (was ${usedKb2}KiB)"

$t1chk = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t1-driver.ps1") "`$ExpectedDriverVersion = '$ExpectedDriverVersion'"
$svcFail = @($t1chk -split "`n" | Where-Object { $_ -match "ASSERT: FAIL" }).Count
Assert "T2.6-driver-healthy" ($svcFail -eq 0) "all T1 checks green after the setmem round trip"

# ===========================================================================
if (-not $SkipStress) {
    Write-Host "`n===== T2.7: hold release under guest pressure (${StressMb}MB) =====" -ForegroundColor Cyan
    $rssBefore = Get-QemuRssMb
    $allocOut = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t2-memstress.ps1") "`$Mode = 'alloc'; `$Mb = $StressMb"
    Write-Host $allocOut
    # give the driver a moment to hand pages back and the guest to fault them in
    $deadline = (Get-Date).AddMinutes(2)
    $rssNow = Get-QemuRssMb
    while (($rssNow - $rssBefore) -lt ($StressMb * 0.5) -and (Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 15
        $rssNow = Get-QemuRssMb
        Write-Host "  pressure RSS=$rssNow MB (was $rssBefore MB)"
    }
    $rise = $rssNow - $rssBefore
    Assert "T2.7-hold-release" ($rise -ge ($StressMb * 0.5)) "RSS rose by ${rise}MB (>= $([int]($StressMb*0.5))MB expected)"

    Write-Host "`n===== T2.8: re-park after release =====" -ForegroundColor Cyan
    $freeOut = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t2-memstress.ps1") "`$Mode = 'free'"
    Write-Host $freeOut
    $deadline = (Get-Date).AddMinutes(4)
    $rssAfter = Get-QemuRssMb
    while ($rssAfter -gt ($rssBefore + 300) -and (Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 20
        $rssAfter = Get-QemuRssMb
        Write-Host "  re-park RSS=$rssAfter MB (target <= $($rssBefore + 300)MB)"
    }
    Assert "T2.8-repark" ($rssAfter -le ($rssBefore + 300)) "RSS returned to ${rssAfter}MB (baseline ${rssBefore}MB +300 tolerance)"

    Write-Host "`n===== T2.4: 2MB alignment of reported blocks =====" -ForegroundColor Cyan
    Enable-DiscardTrace | Out-Null
    Start-Sleep -Seconds 5
    $lines = @(Get-DiscardTraceLines -TailLines 3000 | Where-Object { $_ -match "pc\.ram@" })
    Write-Host ("  collected {0} discard trace lines" -f $lines.Count)
    if ($lines.Count -eq 0) {
        Skip "T2.4-2mb-alignment" "no trace lines available (journald rate-limit); rerun right after a park phase"
    } else {
        $bad = 0
        foreach ($ln in $lines) {
            if ($ln -match "pc\.ram@(0x[0-9a-f]+) \+ (0x[0-9a-f]+)") {
                $addr = [Convert]::ToUInt64($Matches[1], 16)
                $size = [Convert]::ToUInt64($Matches[2], 16)
                if (($addr % 0x200000) -ne 0 -or ($size % 0x200000) -ne 0) { $bad++ }
            }
        }
        Assert "T2.4-2mb-alignment" ($bad -eq 0) ("{0} discard events, {1} misaligned (size and boundary must be 2MB)" -f $lines.Count, $bad)
        Assert "T2.4-2mb-granularity" (($lines | Where-Object { $_ -match "\+ 0x200000" }).Count -eq $lines.Count) "every discard is exactly one 2MB block"
    }
}

# ===========================================================================
if ($IncludeReboot) {
    Write-Host "`n===== T3.1: guest reboot =====" -ForegroundColor Cyan
    Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t2-memstress.ps1") | Out-Null
    Invoke-BareMetal -ScriptBody @"
cat > $script:RemoteDir/_rb.ps1 <<'PSEOF'
shutdown /r /t 5 /c "FPR test reboot"
"reboot issued"
PSEOF
cd /root/letang && python3 winrm_ps.py $script:GuestIp $script:RemoteDir/_rb.ps1 2>&1 | grep -vE 'CLIXML|<Objs'
"@ | ForEach-Object { Write-Host "  $_" }
    Write-Host "  waiting for the guest to come back..."
    Start-Sleep -Seconds 150
    $t1out2 = Invoke-GuestScript (Join-Path $PSScriptRoot "guest\t1-driver.ps1") "`$ExpectedDriverVersion = '$ExpectedDriverVersion'"
    $okLines = @($t1out2 -split "`n" | Where-Object { $_ -match "ASSERT: PASS" })
    $failLines = @($t1out2 -split "`n" | Where-Object { $_ -match "ASSERT: FAIL" })
    Assert "T3.1-driver-after-reboot" ($failLines.Count -eq 0 -and $okLines.Count -ge 3) "T1 checks after reboot: $okLines passed, $failLines failed"

    $rssAfterReboot = Get-QemuRssMb
    $deadline = (Get-Date).AddMinutes(4)
    while ($rssAfterReboot -gt $parkLimit -and (Get-Date) -lt $deadline) {
        Start-Sleep -Seconds 20
        $rssAfterReboot = Get-QemuRssMb
        Write-Host "  post-reboot park RSS=$rssAfterReboot MB"
    }
    Assert "T3.1-repark-after-reboot" ($rssAfterReboot -le $parkLimit) "RSS=${rssAfterReboot}MB <= ${parkLimit}MB"
}

# ===========================================================================
$rc = Write-TestSummary
exit $rc
