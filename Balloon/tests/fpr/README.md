# Free Page Reporting (FPR) test suite for the virtio balloon driver

Automated tests for `VIRTIO_BALLOON_F_PAGE_REPORTING` (2MB-aligned,
report-then-hold implementation in `Balloon/sys/reporting.c`).

## Layout

```
run-fpr-tests.ps1        entry point, runs on the workstation (PowerShell 5.1+)
host/lib-host.ps1        helpers: bare-metal SSH (script-file based), guest
                         WinRM, QMP queries, RSS sampling, trace collection
guest/t1-driver.ps1      guest-side assertions: device, service, driver version
guest/t2-memstress.ps1   guest-side memory stress (allocate/touch/release)
```

## Usage

```powershell
# full run against the default test VM on the bare metal
.\run-fpr-tests.ps1

# typical parameters
.\run-fpr-tests.ps1 -BareMetal 47.83.225.42 -VmName fpr_upstream `
    -GuestIp 192.168.122.6 -ExpectedDriverVersion 100.6.101.58300

# include the reboot test (takes ~4 extra minutes)
.\run-fpr-tests.ps1 -IncludeReboot

# quick smoke run without the stress cycle
.\run-fpr-tests.ps1 -SkipStress
```

Prerequisites:
- workstation can SSH to the bare metal as root (key auth),
- the test VM runs with `<memballoon model='virtio' freePageReporting='on'/>`,
- the guest has WinRM enabled (winrm quickconfig) and the test driver
  installed; the balloon device is the modern virtio device (DEV_1045),
- on the bare metal: `/root/letang/winrm_ps.py` (pywinrm) must exist.

## Test matrix

| id  | area                  | assertion                                              | level |
|-----|-----------------------|--------------------------------------------------------|-------|
| T1.1| device                | modern balloon device present, Status OK               | auto  |
| T1.2| service               | BALLOON service RUNNING                                | auto  |
| T1.3| driver                | driver version matches -ExpectedDriverVersion          | auto  |
| T2.1| negotiation           | QEMU sees VIRTIO_BALLOON_F_REPORTING in guest-features | auto  |
| T2.2| park                  | QEMU RSS drops below 60% of VM RAM                     | auto  |
| T2.3| hold                  | RSS stable for 3 min (rebound < 10% of parked amount)  | auto  |
| T2.4| 2MB alignment         | every ram_block_discard_range trace line is 2MB-aligned in size and boundary | auto (SKIP if journald rate-limited) |
| T2.5| stats regression      | QEMU balloon stats readable, total == VM RAM           | auto  |
| T2.6| inflate/deflate regr. | virsh setmem + guest visible memory change, no crash   | auto  |
| T2.7| hold release          | guest allocates 1.5GB, QEMU RSS rises again            | auto  |
| T2.8| re-park               | after release RSS falls back to baseline within 3 min  | auto  |
| T3.1| reboot                | after guest reboot: T1 green, park works again         | auto (-IncludeReboot) |
| M.1 | driver verifier       | enable DV for balloon.sys, rerun T2, no bugcheck       | manual |
| M.2 | surprise removal      | virsh detach-device, guest survives, pages released    | manual |
| M.3 | soak                  | 12h idle + periodic stress, RSS sawtooth, no leak      | manual |
| M.4 | other guest images    | repeat on Win10/Win11 client images                    | manual |

Notes:
- T2.4 depends on the QEMU simple/log trace reaching journald; heavy
  trace traffic may be rate-limited by journald. The test re-arms the
  trace event and triggers a fresh park before sampling, and reports
  SKIP (not FAIL) when no trace lines can be collected.
- RSS assertions use the QEMU process RSS (`/proc/<pid>/status`),
  i.e. actual host-side memory occupancy - the quantity FPR is meant
  to reduce.
