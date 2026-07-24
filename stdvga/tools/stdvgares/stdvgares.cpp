#include <windows.h>
#include <userenv.h>
#include <wtsapi32.h>

#include <cwchar>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr DWORD DmPelsWidth = DM_PELSWIDTH;
constexpr DWORD DmPelsHeight = DM_PELSHEIGHT;
constexpr DWORD InvalidSessionId = 0xFFFFFFFF;

void PrintUsage()
{
    std::wcout << L"Usage: stdvgares.exe <width> <height>\n"
               << L"\n"
               << L"Examples:\n"
               << L"  stdvgares.exe 1280 720\n"
               << L"  stdvgares.exe 1920 1080\n";
}

bool ParseUInt(const wchar_t *text, DWORD &value)
{
    if (text == nullptr || *text == L'\0')
    {
        return false;
    }

    wchar_t *end = nullptr;
    unsigned long parsed = std::wcstoul(text, &end, 10);
    if (end == text || *end != L'\0' || parsed == 0 || parsed > 65535)
    {
        return false;
    }

    value = static_cast<DWORD>(parsed);
    return true;
}

std::wstring QuoteArgument(const std::wstring &arg)
{
    if (arg.empty())
    {
        return L"\"\"";
    }

    bool needsQuotes = false;
    for (wchar_t ch : arg)
    {
        if (ch == L' ' || ch == L'\t' || ch == L'\"')
        {
            needsQuotes = true;
            break;
        }
    }

    if (!needsQuotes)
    {
        return arg;
    }

    std::wstring quoted = L"\"";
    for (wchar_t ch : arg)
    {
        if (ch == L'\"')
        {
            quoted += L'\\';
        }
        quoted += ch;
    }
    quoted += L"\"";
    return quoted;
}

bool IsSessionZero()
{
    DWORD sessionId = InvalidSessionId;
    if (!ProcessIdToSessionId(GetCurrentProcessId(), &sessionId))
    {
        return false;
    }

    return sessionId == 0;
}

std::wstring GetSelfPath()
{
    std::wstring path(MAX_PATH, L'\0');
    DWORD len = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
    if (len == 0)
    {
        return L"stdvgares.exe";
    }

    path.resize(len);
    return path;
}

std::wstring BuildCommandLine(const std::wstring &selfPath, const std::vector<std::wstring> &args)
{
    std::wstring commandLine = QuoteArgument(selfPath);
    commandLine += L" /_session_child";
    for (const auto &arg : args)
    {
        commandLine += L' ';
        commandLine += QuoteArgument(arg);
    }
    return commandLine;
}

void RelayFileToStdout(const std::wstring &path)
{
    HANDLE file = CreateFileW(path.c_str(),
                              GENERIC_READ,
                              FILE_SHARE_READ | FILE_SHARE_WRITE,
                              nullptr,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL,
                              nullptr);
    if (file == INVALID_HANDLE_VALUE)
    {
        return;
    }

    HANDLE output = GetStdHandle(STD_OUTPUT_HANDLE);
    char buffer[4096];
    DWORD read = 0;
    while (ReadFile(file, buffer, sizeof(buffer), &read, nullptr) && read != 0)
    {
        DWORD written = 0;
        WriteFile(output, buffer, read, &written, nullptr);
    }

    CloseHandle(file);
}

DWORD FindActiveSessionId()
{
    PWTS_SESSION_INFOW sessions = nullptr;
    DWORD sessionCount = 0;
    DWORD activeSessionId = InvalidSessionId;
    bool sessionsEnumerated = false;

    if (WTSEnumerateSessionsW(WTS_CURRENT_SERVER_HANDLE, 0, 1, &sessions, &sessionCount))
    {
        sessionsEnumerated = true;
        for (DWORD index = 0; index < sessionCount; ++index)
        {
            if (sessions[index].State == WTSActive && sessions[index].SessionId != 0)
            {
                activeSessionId = sessions[index].SessionId;
                break;
            }
        }
        WTSFreeMemory(sessions);
    }

    if (activeSessionId != InvalidSessionId)
    {
        return activeSessionId;
    }

    if (sessionsEnumerated)
    {
        return InvalidSessionId;
    }

    activeSessionId = WTSGetActiveConsoleSessionId();
    return activeSessionId == 0 ? InvalidSessionId : activeSessionId;
}

int RunInActiveSession(const std::vector<std::wstring> &args)
{
    DWORD activeSessionId = FindActiveSessionId();
    if (activeSessionId == InvalidSessionId || activeSessionId == 0)
    {
        std::wcerr << L"No active user session is available.\n";
        return ERROR_CTX_WINSTATION_NOT_FOUND;
    }

    HANDLE userToken = nullptr;
    if (!WTSQueryUserToken(activeSessionId, &userToken))
    {
        DWORD error = GetLastError();
        std::wcerr << L"WTSQueryUserToken failed, GetLastError=" << error << L"\n";
        return static_cast<int>(error);
    }

    wchar_t tempPath[MAX_PATH] = {};
    wchar_t tempFile[MAX_PATH] = {};
    GetTempPathW(_countof(tempPath), tempPath);
    GetTempFileNameW(tempPath, L"svg", 0, tempFile);

    SECURITY_ATTRIBUTES sa = {};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    HANDLE childOutput = CreateFileW(tempFile,
                                     GENERIC_WRITE,
                                     FILE_SHARE_READ | FILE_SHARE_WRITE,
                                     &sa,
                                     CREATE_ALWAYS,
                                     FILE_ATTRIBUTE_NORMAL,
                                     nullptr);
    if (childOutput == INVALID_HANDLE_VALUE)
    {
        DWORD error = GetLastError();
        CloseHandle(userToken);
        std::wcerr << L"CreateFile for child output failed, GetLastError=" << error << L"\n";
        return static_cast<int>(error);
    }

    void *environment = nullptr;
    CreateEnvironmentBlock(&environment, userToken, FALSE);

    STARTUPINFOW si = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES;
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdOutput = childOutput;
    si.hStdError = childOutput;

    PROCESS_INFORMATION pi = {};
    std::wstring commandLine = BuildCommandLine(GetSelfPath(), args);
    std::vector<wchar_t> mutableCommandLine(commandLine.begin(), commandLine.end());
    mutableCommandLine.push_back(L'\0');

    BOOL created = CreateProcessAsUserW(userToken,
                                        nullptr,
                                        mutableCommandLine.data(),
                                        nullptr,
                                        nullptr,
                                        TRUE,
                                        CREATE_UNICODE_ENVIRONMENT | CREATE_NO_WINDOW,
                                        environment,
                                        nullptr,
                                        &si,
                                        &pi);

    DWORD createError = GetLastError();
    CloseHandle(childOutput);
    if (environment != nullptr)
    {
        DestroyEnvironmentBlock(environment);
    }
    CloseHandle(userToken);

    if (!created)
    {
        DeleteFileW(tempFile);
        std::wcerr << L"CreateProcessAsUserW failed, GetLastError=" << createError << L"\n";
        return static_cast<int>(createError);
    }

    WaitForSingleObject(pi.hProcess, INFINITE);

    DWORD exitCode = 0;
    GetExitCodeProcess(pi.hProcess, &exitCode);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);

    RelayFileToStdout(tempFile);
    DeleteFileW(tempFile);

    return static_cast<int>(exitCode);
}

const wchar_t *DispChangeToString(LONG result)
{
    switch (result)
    {
        case DISP_CHANGE_SUCCESSFUL:
            return L"DISP_CHANGE_SUCCESSFUL";
        case DISP_CHANGE_RESTART:
            return L"DISP_CHANGE_RESTART";
        case DISP_CHANGE_FAILED:
            return L"DISP_CHANGE_FAILED";
        case DISP_CHANGE_BADMODE:
            return L"DISP_CHANGE_BADMODE";
        case DISP_CHANGE_NOTUPDATED:
            return L"DISP_CHANGE_NOTUPDATED";
        case DISP_CHANGE_BADFLAGS:
            return L"DISP_CHANGE_BADFLAGS";
        case DISP_CHANGE_BADPARAM:
            return L"DISP_CHANGE_BADPARAM";
        case DISP_CHANGE_BADDUALVIEW:
            return L"DISP_CHANGE_BADDUALVIEW";
        default:
            return L"DISP_CHANGE_UNKNOWN";
    }
}

int ResultToExitCode(LONG result)
{
    if (result == DISP_CHANGE_SUCCESSFUL || result == DISP_CHANGE_RESTART)
    {
        return 0;
    }

    return result < 0 ? static_cast<int>(-result) : static_cast<int>(result);
}

//
// Try to set the resolution via the modern SetDisplayConfig API.
// SetDisplayConfig does not trigger the Windows Shell "keep these display
// settings?" confirmation dialog that CDS_UPDATEREGISTRY causes in
// interactive sessions, making it the preferred path for both interactive
// and unattended use.
//
// Returns ERROR_SUCCESS on success, or a Win32 error code on failure.
// Callers should fall back to ChangeDisplaySettingsExW on failure.
//
LONG SetResolutionViaSetDisplayConfig(const std::wstring &gdiDeviceName, DWORD width, DWORD height)
{
    UINT32 numPaths = 0;
    UINT32 numModes = 0;
    LONG res = GetDisplayConfigBufferSizes(QDC_ONLY_ACTIVE_PATHS, &numPaths, &numModes);
    if (res != ERROR_SUCCESS)
    {
        return res;
    }

    std::vector<DISPLAYCONFIG_PATH_INFO> paths(numPaths);
    std::vector<DISPLAYCONFIG_MODE_INFO> modes(numModes);
    res = QueryDisplayConfig(QDC_ONLY_ACTIVE_PATHS, &numPaths, paths.data(), &numModes, modes.data(), nullptr);
    if (res != ERROR_SUCCESS)
    {
        return res;
    }
    paths.resize(numPaths);
    modes.resize(numModes);

    // Find the DISPLAYCONFIG path that corresponds to our QEMU VGA device.
    bool found = false;
    for (UINT32 i = 0; i < static_cast<UINT32>(paths.size()); i++)
    {
        DISPLAYCONFIG_SOURCE_DEVICE_NAME srcName = {};
        srcName.header.type = DISPLAYCONFIG_DEVICE_INFO_GET_SOURCE_NAME;
        srcName.header.size = sizeof(srcName);
        srcName.header.adapterId = paths[i].sourceInfo.adapterId;
        srcName.header.id = paths[i].sourceInfo.id;
        if (DisplayConfigGetDeviceInfo(&srcName.header) != ERROR_SUCCESS)
        {
            continue;
        }

        if (_wcsicmp(srcName.viewGdiDeviceName, gdiDeviceName.c_str()) != 0)
        {
            continue;
        }

        // Update the source mode for this path.
        UINT32 srcIdx = paths[i].sourceInfo.modeInfoIdx;
        if (srcIdx != DISPLAYCONFIG_PATH_MODE_IDX_INVALID && srcIdx < static_cast<UINT32>(modes.size()) &&
            modes[srcIdx].infoType == DISPLAYCONFIG_MODE_INFO_TYPE_SOURCE)
        {
            modes[srcIdx].sourceMode.width = width;
            modes[srcIdx].sourceMode.height = height;
            found = true;
        }

        // Also update the target mode's video signal size. Without this,
        // Windows can silently keep the previous effective resolution when
        // the requested aspect ratio differs from what the target mode
        // currently advertises (e.g. switching between 16:9 and 4:3), even
        // though SetDisplayConfig still reports success. Force identity
        // scaling so the GPU does not try to letterbox/stretch the new
        // source mode to the old target signal size.
        UINT32 tgtIdx = paths[i].targetInfo.modeInfoIdx;
        if (tgtIdx != DISPLAYCONFIG_PATH_MODE_IDX_INVALID && tgtIdx < static_cast<UINT32>(modes.size()) &&
            modes[tgtIdx].infoType == DISPLAYCONFIG_MODE_INFO_TYPE_TARGET)
        {
            DISPLAYCONFIG_VIDEO_SIGNAL_INFO &signal = modes[tgtIdx].targetMode.targetVideoSignalInfo;
            signal.activeSize.cx = width;
            signal.activeSize.cy = height;
            signal.totalSize.cx = width;
            signal.totalSize.cy = height;
            signal.AdditionalSignalInfo.vSyncFreqDivider = 1;
            signal.videoStandard = 255; // D3DKMDT_VSS_OTHER: no fixed timing standard
            signal.scanLineOrdering = DISPLAYCONFIG_SCANLINE_ORDERING_PROGRESSIVE;
        }
        paths[i].targetInfo.scaling = DISPLAYCONFIG_SCALING_IDENTITY;
        break;
    }

    if (!found)
    {
        return ERROR_NOT_FOUND;
    }

    UINT32 flags = SDC_APPLY | SDC_SAVE_TO_DATABASE | SDC_ALLOW_CHANGES | SDC_USE_SUPPLIED_DISPLAY_CONFIG;
    return SetDisplayConfig(static_cast<UINT32>(paths.size()),
                            paths.data(),
                            static_cast<UINT32>(modes.size()),
                            modes.data(),
                            flags);
}

bool ContainsInsensitive(const wchar_t *text, const wchar_t *needle)
{
    if (text == nullptr || needle == nullptr || *needle == L'\0')
    {
        return false;
    }

    size_t needleLength = std::wcslen(needle);
    for (const wchar_t *current = text; *current != L'\0'; ++current)
    {
        if (_wcsnicmp(current, needle, needleLength) == 0)
        {
            return true;
        }
    }

    return false;
}

struct DisplayTarget
{
    std::wstring deviceName;
    std::wstring deviceString;
    std::wstring deviceId;
};

bool IsStdVgaDisplayDevice(const DISPLAY_DEVICEW &device)
{
    return ContainsInsensitive(device.DeviceID, L"VEN_1234&DEV_1111") ||
           ContainsInsensitive(device.DeviceString, L"QEMU");
}

bool FindStdVgaDisplayTarget(DisplayTarget &target)
{
    for (DWORD index = 0; index < 32; ++index)
    {
        DISPLAY_DEVICEW device = {};
        device.cb = sizeof(device);
        if (!EnumDisplayDevicesW(nullptr, index, &device, 0))
        {
            break;
        }

        if ((device.StateFlags & DISPLAY_DEVICE_ATTACHED_TO_DESKTOP) == 0)
        {
            continue;
        }

        if (!IsStdVgaDisplayDevice(device))
        {
            continue;
        }

        target.deviceName = device.DeviceName;
        target.deviceString = device.DeviceString;
        target.deviceId = device.DeviceID;
        return true;
    }

    return false;
}

int ChangeResolution(const std::vector<std::wstring> &args)
{
    if (args.size() == 1 && (_wcsicmp(args[0].c_str(), L"/?") == 0 || _wcsicmp(args[0].c_str(), L"/help") == 0))
    {
        PrintUsage();
        return 0;
    }

    if (args.size() != 2)
    {
        PrintUsage();
        return 2;
    }

    DWORD width = 0;
    DWORD height = 0;

    if (!ParseUInt(args[0].c_str(), width) || !ParseUInt(args[1].c_str(), height))
    {
        std::wcerr << L"Invalid width or height.\n";
        PrintUsage();
        return 2;
    }

    DisplayTarget target;
    if (!FindStdVgaDisplayTarget(target))
    {
        std::wcerr << L"QEMU VGA display device was not found.\n";
        return ERROR_NOT_FOUND;
    }

    std::wcout << L"Target display: ";
    if (!target.deviceString.empty())
    {
        std::wcout << target.deviceString << L" ";
    }
    std::wcout << L"(" << target.deviceName << L")\n";

    DEVMODEW mode = {};
    mode.dmSize = sizeof(mode);
    if (!EnumDisplaySettingsExW(target.deviceName.c_str(), ENUM_CURRENT_SETTINGS, &mode, 0))
    {
        DWORD error = GetLastError();
        std::wcerr << L"EnumDisplaySettingsExW failed, GetLastError=" << error << L"\n";
        return static_cast<int>(error == ERROR_SUCCESS ? ERROR_INVALID_DATA : error);
    }

    std::wcout << L"Mode before change: " << mode.dmPelsWidth << L"x" << mode.dmPelsHeight << L"\n";

    std::wcout << L"Requested mode: " << width << L"x" << height << L"\n";

    //
    // Prefer SetDisplayConfig: it drives the same VidPn re-negotiation
    // (DxgkDdiEnumVidPnCofuncModality / DxgkDdiCommitVidPn) without
    // triggering the Windows Shell "keep these display settings?" dialog
    // that CDS_UPDATEREGISTRY causes. Fall back to ChangeDisplaySettingsExW
    // if SetDisplayConfig cannot find or update the display path.
    //
    LONG sdcResult = SetResolutionViaSetDisplayConfig(target.deviceName, width, height);
    if (sdcResult == ERROR_SUCCESS)
    {
        std::wcout << L"SetDisplayConfig: success\n";
    }
    else
    {
        std::wcout << L"SetDisplayConfig returned " << sdcResult << L", falling back to ChangeDisplaySettingsExW\n";

        mode.dmFields |= DmPelsWidth | DmPelsHeight;
        mode.dmPelsWidth = width;
        mode.dmPelsHeight = height;

        // CDS_UPDATEREGISTRY persists this mode as the user's requested
        // display setting (survives logoff/reboot and is reflected by
        // Display Settings). It has no effect on the stdvga miniport itself:
        // the actual mode change is driven purely by the dxgkrnl VidPn
        // re-negotiation that ChangeDisplaySettingsExW triggers, independent
        // of this flag.
        LONG result = ChangeDisplaySettingsExW(target.deviceName.c_str(), &mode, nullptr, CDS_UPDATEREGISTRY, nullptr);
        std::wcout << L"ChangeDisplaySettingsExW returned " << result << L" (" << DispChangeToString(result) << L")\n";

        if (ResultToExitCode(result) != 0)
        {
            return ResultToExitCode(result);
        }
    }

    DEVMODEW current = {};
    current.dmSize = sizeof(current);
    if (EnumDisplaySettingsExW(target.deviceName.c_str(), ENUM_CURRENT_SETTINGS, &current, 0))
    {
        std::wcout << L"Mode after change: " << current.dmPelsWidth << L"x" << current.dmPelsHeight << L"\n";
    }

    return 0;
}

} // namespace

int wmain(int argc, wchar_t **argv)
{
    bool sessionChild = false;
    std::vector<std::wstring> args;

    for (int i = 1; i < argc; ++i)
    {
        if (_wcsicmp(argv[i], L"/_session_child") == 0)
        {
            sessionChild = true;
            continue;
        }
        args.emplace_back(argv[i]);
    }

    if (!sessionChild && IsSessionZero())
    {
        return RunInActiveSession(args);
    }

    return ChangeResolution(args);
}
