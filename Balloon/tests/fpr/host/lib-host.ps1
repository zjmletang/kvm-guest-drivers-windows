# Host-side helper library for the FPR test suite.
# All bare-metal interaction goes through temporary script files uploaded
# over SCP (direct inline SSH quoting proved unreliable), guest interaction
# goes through pywinrm (winrm_ps.py) on the bare metal.

$script:BareMetal = "47.83.225.42"
$script:VmName = "fpr_upstream"
$script:GuestIp = "192.168.122.6"
$script:GuestUser = "Administrator"
$script:GuestPass = "123456"
$script:RemoteDir = "/root/letang/fpr_tests"

# ---------------------------------------------------------------------------
# assertion bookkeeping (Constrained Language Mode compatible: plain strings)
# ---------------------------------------------------------------------------
$script:TestResults = @()

function Assert {
    param([string]$Name, [bool]$Condition, [string]$Detail)
    $script:TestResults += ("{0}|{1}|{2}" -f $(if ($Condition) { "PASS" } else { "FAIL" }), $Name, $Detail)
    $tag = if ($Condition) { "PASS" } else { "FAIL" }
    Write-Host ("  [{0}] {1} - {2}" -f $tag, $Name, $Detail)
}

function Skip {
    param([string]$Name, [string]$Reason)
    $script:TestResults += ("SKIP|{0}|{1}" -f $Name, $Reason)
    Write-Host ("  [SKIP] {0} - {1}" -f $Name, $Reason)
}

function Write-TestSummary {
    $failed = @($script:TestResults | Where-Object { $_ -like "FAIL|*" })
    Write-Host ("`n===== SUMMARY: {0} checks, {1} failed =====" -f $script:TestResults.Count, $failed.Count)
    if ($failed.Count -gt 0) {
        $failed | ForEach-Object { Write-Host ("  {0}" -f $_) }
        return 1
    }
    return 0
}

# ---------------------------------------------------------------------------
# bare metal transport (script-file based, with retries)
# ---------------------------------------------------------------------------
function Invoke-BareMetal {
    param([string]$ScriptBody, [int]$Retries = 3, [int]$TimeoutSec = 300)
    $local = Join-Path $env:TEMP ("fprt_{0}.sh" -f (Get-Random))
    $remote = "$script:RemoteDir/" + (Split-Path -Leaf $local)
    # write LF line endings, bash cannot parse CRLF scripts
    Set-Content -Path $local -Value ($ScriptBody -replace "`r`n", "`n") -Encoding Ascii -NoNewline
    try {
        for ($i = 1; $i -le $Retries; $i++) {
            scp -o ConnectTimeout=10 -o StrictHostKeyChecking=no $local "root@$script:BareMetal`:$remote" 2>$null
            if ($LASTEXITCODE -ne 0) { Start-Sleep -Seconds 5; continue }
            $out = ssh -o ConnectTimeout=10 -o StrictHostKeyChecking=no root@$script:BareMetal "timeout $TimeoutSec bash $remote" 2>$null
            if ($LASTEXITCODE -eq 0) { return ($out -join "`n") }
            Start-Sleep -Seconds 5
        }
        throw "bare metal command failed after $Retries retries: $ScriptBody"
    } finally {
        Remove-Item $local -ErrorAction SilentlyContinue
    }
}

# run one of the guest scripts (by local path) inside the guest via WinRM.
# winrm_ps.py sends the *content* of a script file to the guest, so the
# parameters are prepended as variable assignments and the guest script
# itself is concatenated into a combined script on the bare metal.
function Invoke-GuestScript {
    param([string]$LocalPath, [string]$Arguments = "")
    $remote = "$script:RemoteDir/" + (Split-Path -Leaf $LocalPath)
    $uploaded = $false
    for ($i = 1; $i -le 3; $i++) {
        scp -o ConnectTimeout=10 -o StrictHostKeyChecking=no $LocalPath "root@$script:BareMetal`:$remote" 2>$null | Out-Null
        if ($LASTEXITCODE -eq 0) { $uploaded = $true; break }
        Start-Sleep -Seconds 5
    }
    if (-not $uploaded) { throw "scp of $LocalPath failed after 3 retries" }
    $argLine = ""
    if ($Arguments -ne "") { $argLine = $Arguments }
    $body = @"
cat > $script:RemoteDir/_combined.ps1 <<'PSEOF'
$argLine
PSEOF
cat $remote >> $script:RemoteDir/_combined.ps1
cd /root/letang
python3 winrm_ps.py $script:GuestIp $script:RemoteDir/_combined.ps1 2>&1 | sed -e 's/^<Objs .*<\/Objs>//' -e 's/#< CLIXML//' | grep -vE '^<Objs |^$' || true
"@
    return (Invoke-BareMetal -ScriptBody $body -TimeoutSec 420)
}

# ---------------------------------------------------------------------------
# QEMU side helpers (executed on the bare metal)
# ---------------------------------------------------------------------------
function Get-QemuRssMb {
    # returns the QEMU process RSS of the test VM in MB, or -1
    $body = @"
QPID=`$(pgrep -f "$script:VmName.*qcow2" | head -1)
[ -z "`$QPID" ] && QPID=`$(pgrep -f "name guest=$script:VmName" | head -1)
if [ -z "`$QPID" ]; then echo -1; else grep VmRSS /proc/`$QPID/status | awk '{print int(`$2/1024)}'; fi
"@
    $r = Invoke-BareMetal -ScriptBody $body
    $v = $r.Trim() -as [int]
    if ($null -ne $v) { return $v }
    return -1
}

function Get-BalloonGuestFeatures {
    $body = @"
virsh qemu-monitor-command $script:VmName --pretty '{"execute":"x-query-virtio-status","arguments":{"path":"/machine/peripheral/balloon0/virtio-backend"}}' 2>&1 | grep -A8 'guest-features'
"@
    return (Invoke-BareMetal -ScriptBody $body)
}

function Get-BalloonInfo {
    # HMP info balloon (stats if the stats feature is active)
    $body = "virsh qemu-monitor-command $script:VmName --hmp 'info balloon' 2>&1"
    return (Invoke-BareMetal -ScriptBody $body)
}

function Enable-DiscardTrace {
    $body = "virsh qemu-monitor-command $script:VmName --hmp 'trace-event ram_block_discard_range on' 2>&1; echo ARMED"
    return (Invoke-BareMetal -ScriptBody $body)
}

function Get-DiscardTraceLines {
    param([int]$TailLines = 2000)
    $body = "journalctl --no-pager -n $TailLines -o cat 2>/dev/null | grep ram_block_discard_range"
    return (Invoke-BareMetal -ScriptBody $body)
}
