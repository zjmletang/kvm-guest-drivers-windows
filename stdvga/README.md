# QEMU Standard VGA Display Driver for Windows

A Windows Kernel Mode Display-Only Driver (KMDOD) for the QEMU Standard VGA device (PCI VEN_1234 / DEV_1111).

Windows UEFI guests using the default Microsoft Basic Display Adapter are stuck at a fixed resolution because there is no VGA BIOS to provide mode setting. This driver programs the Bochs VBE registers directly, enabling runtime resolution switching on both BIOS and UEFI boot modes.

## Features

- Runtime resolution switching via Windows Display Settings, `Set-DisplayResolution`, or `stdvgares.exe`
- 15 supported resolutions: 800x600 (default), 1024x768, 1152x864, 1280x720, 1280x768, 1280x800, 1280x960, 1280x1024, 1400x1050, 1440x900, 1600x1200, 1680x1050, 1920x1080, 1920x1200, 2560x1600
- Single Universal x64 package covering Windows 10 (1709+) / 11 and Windows Server 2019 / 2022 / 2025

## Installation

```cmd
pnputil /add-driver stdvga.inf /install
```

Uninstall:

```cmd
pnputil /delete-driver oemXX.inf /uninstall /force
```

## Usage

On Windows Server, the built-in PowerShell cmdlet also works:

```powershell
Set-DisplayResolution -Width 1920 -Height 1080 -Force
```

For unattended environments, including callers running in Session 0, use the
helper tool built with the solution:

```cmd
stdvgares.exe 1280 720
stdvgares.exe 1920 1080
```

The tool can be launched from Session 0 and automatically runs the resolution
change in the active user session. This simplifies unattended setup, but it does
not remove the Windows requirement for an active interactive session: if no user
session is active, the resolution cannot be changed.

## License

[BSD 3-Clause](LICENSE) — Copyright (c) 2026 Alibaba Cloud Computing Ltd.
