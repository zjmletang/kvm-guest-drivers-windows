# QEMU Standard VGA Display Driver for Windows

A Windows Kernel Mode Display-Only Driver (KMDOD) for the QEMU Standard VGA device (PCI VEN_1234 / DEV_1111).

Windows UEFI guests using the default Microsoft Basic Display Adapter are stuck at a fixed resolution because there is no VGA BIOS to provide mode setting. This driver programs the Bochs VBE registers directly, enabling runtime resolution switching on both BIOS and UEFI boot modes.

## Features

- Runtime resolution switching via Windows Display Settings, `Set-DisplayResolution`, or `stdvgares.exe`
- 15 supported resolutions: 800x600, 1024x768, 1152x864, 1280x720, 1280x768, 1280x800, 1280x960, 1280x1024, 1400x1050, 1440x900, 1600x1200, 1680x1050, 1920x1080 (default), 1920x1200, 2560x1600
- Single Universal x64 package covering Windows 10 (1709+) / 11 and Windows Server 2019 / 2022 / 2025

## Building

This driver is part of the [kvm-guest-drivers-windows](https://github.com/virtio-win/kvm-guest-drivers-windows) project and uses its common build system.

```cmd
cd stdvga
buildAll.bat
```

You can also build directly with Visual Studio 2022 / MSBuild:

```cmd
msbuild stdvga.sln /p:Configuration="Win10 Release" /p:Platform=x64
```

Driver output is placed in `Install\Win10\amd64\` (`stdvga.sys` / `stdvga.inf` / `stdvga.cat`). The helper tool is built as `tools\stdvgares\objfre_win10_amd64\amd64\stdvgares.exe`.

> The solution also exposes a `Win11 Release` configuration (`buildAll.bat Win11`), which produces a byte-identical Universal Driver in `Install\Win11\amd64\`. It is reserved for future Windows 11–only DDI work and is **not** required for normal builds or releases — a single package built from `Win10 Release` is the supported artifact.

## Installation

```cmd
pnputil /add-driver stdvga.inf /install
```

Uninstall:

```cmd
pnputil /delete-driver oemXX.inf /uninstall /force
```

## Usage

Interactive (VNC / RDP):

```powershell
Set-DisplayResolution -Width 1920 -Height 1080 -Force
```

For automated or client-SKU scenarios, use the helper tool built with the solution:

```cmd
stdvgares.exe 1280 720
stdvgares.exe 1920 1080
```

The helper calls `ChangeDisplaySettingsExW` and returns zero for successful mode changes. It automatically targets the QEMU VGA display device, so it does not require a device argument when another virtual display adapter is present. If it is launched from session 0, it automatically starts itself in the active user session and applies the requested resolution there. It also prints the target display, current mode, requested mode, return code, and observed mode for automation logs.

## License

[BSD 3-Clause](LICENSE) — Copyright (c) 2026 Alibaba Cloud Computing Ltd.
