# Guest-side memory stress for FPR hold-release/re-park tests.
# Executed inside the guest via WinRM (script content is sent, not the file).
# Parameters are injected as variables prepended to this script:
#   $Mode - "alloc" | "free" | "query"
#   $Mb   - megabytes to allocate in alloc mode
#     alloc: allocate and touch $Mb MB in a background job that keeps running,
#            then report guest free memory
#     free:  stop the background job and force GC, report free memory
#     query: report current total/free memory (KB)
if (-not $Mode) { $Mode = "query" }
if (-not $Mb)   { $Mb = 1500 }

switch ($Mode) {
    "alloc" {
        $job = Start-Job -ScriptBlock {
            param($mb)
            $b = New-Object byte[] ($mb * 1MB)
            for ($i = 0; $i -lt $b.Length; $i += 4096) { $b[$i] = 1 }
            "TOUCHED $mb MB"
            while ($true) { Start-Sleep -Seconds 5 }
        } -ArgumentList $Mb
        $deadline = (Get-Date).AddSeconds(120)
        $touched = $false
        while ((Get-Date) -lt $deadline) {
            Start-Sleep -Seconds 5
            if ((Receive-Job $job -Keep) -match "TOUCHED") { $touched = $true; break }
        }
        if (-not $touched) { Write-Output "WARN: alloc job did not confirm touch within 120s" }
        $free = (Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory
        Write-Output "ALLOC_DONE touched=$touched freeKB=$free"
    }
    "free" {
        Get-Job | Stop-Job -ErrorAction SilentlyContinue
        Get-Job | Remove-Job -Force -ErrorAction SilentlyContinue
        [GC]::Collect()
        [GC]::WaitForPendingFinalizers()
        Start-Sleep -Seconds 3
        $free = (Get-CimInstance Win32_OperatingSystem).FreePhysicalMemory
        Write-Output "FREE_DONE freeKB=$free"
    }
    "query" {
        $os = Get-CimInstance Win32_OperatingSystem
        Write-Output "totalKB=$($os.TotalVisibleMemorySize) freeKB=$($os.FreePhysicalMemory)"
    }
}
