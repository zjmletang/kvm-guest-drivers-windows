# Guest-side assertions: balloon device, service, driver version.
# Executed inside the guest via WinRM (script content is sent, not the file).
# Parameters are injected as variables prepended to this script:
#   $ExpectedDriverVersion - expected driver version string ("" = no check)
$results = @()
function Assert2($Name, $Condition, $Detail) {
    $script:results += ("{0}|{1}|{2}" -f $(if ($Condition) { "PASS" } else { "FAIL" }), $Name, $Detail)
    Write-Output "ASSERT: $(if ($Condition) { 'PASS' } else { 'FAIL' }) $Name - $Detail"
}

if (-not $ExpectedDriverVersion) { $ExpectedDriverVersion = "" }

# T1.1 modern balloon device present and OK
$dev = Get-PnpDevice | Where-Object { $_.FriendlyName -like "*Balloon*" -and $_.InstanceId -like "*DEV_1045*" }
Assert2 "T1.1-device-present" ($null -ne $dev) $(if ($dev) { $dev.InstanceId } else { "no DEV_1045 balloon device" })
Assert2 "T1.1-device-status" ($null -ne $dev -and $dev.Status -eq "OK") $(if ($dev) { "Status=$($dev.Status)" } else { "n/a" })

# T1.2 BALLOON service running
$svc = Get-Service BALLOON -ErrorAction SilentlyContinue
Assert2 "T1.2-service-running" ($null -ne $svc -and $svc.Status -eq "Running") $(if ($svc) { "Status=$($svc.Status)" } else { "service not found" })

# T1.3 driver version
$drv = Get-CimInstance Win32_PnPSignedDriver |
    Where-Object { $_.DeviceName -like "*Balloon*" -and $_.DeviceID -like "*1045*" }
if ($null -eq $drv) {
    $drvPath = (Get-CimInstance Win32_SystemDriver -Filter "Name='BALLOON'").PathName
    Assert2 "T1.3-driver-version" $false "no PnPSignedDriver record, svc path=$drvPath"
} elseif ($ExpectedDriverVersion -ne "") {
    Assert2 "T1.3-driver-version" ($drv.DriverVersion -eq $ExpectedDriverVersion) "actual=$($drv.DriverVersion) expected=$ExpectedDriverVersion inf=$($drv.InfName)"
} else {
    Assert2 "T1.3-driver-version" $true "version=$($drv.DriverVersion) inf=$($drv.InfName)"
}

$fails = @($script:results | Where-Object { $_ -like "FAIL|*" }).Count
Write-Output "T1_SUMMARY: total=$($script:results.Count) failed=$fails"
