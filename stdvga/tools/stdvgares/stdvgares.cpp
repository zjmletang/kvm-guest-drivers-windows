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

    std::wcout << L"Target display: " << target.deviceName;
    if (!target.deviceString.empty())
    {
        std::wcout << L" (" << target.deviceString << L")";
    }
    std::wcout << L"\n";

    DEVMODEW mode = {};
    mode.dmSize = sizeof(mode);
    if (!EnumDisplaySettingsExW(target.deviceName.c_str(), ENUM_CURRENT_SETTINGS, &mode, 0))
    {
        DWORD error = GetLastError();
        std::wcerr << L"EnumDisplaySettingsExW failed, GetLastError=" << error << L"\n";
        return static_cast<int>(error == ERROR_SUCCESS ? ERROR_INVALID_DATA : error);
    }

    std::wcout << L"Current mode: " << mode.dmPelsWidth << L"x" << mode.dmPelsHeight;
    if (mode.dmDisplayFrequency != 0)
    {
        std::wcout << L"@" << mode.dmDisplayFrequency;
    }
    std::wcout << L"\n";

    mode.dmFields |= DmPelsWidth | DmPelsHeight;
    mode.dmPelsWidth = width;
    mode.dmPelsHeight = height;

    // CDS_UPDATEREGISTRY only tells Windows to persist this mode as the
    // user's requested display setting (so it survives logoff/reboot and is
    // reflected by tools such as Display Settings). It has no effect on the
    // stdvga miniport itself: the actual mode change is driven purely by the
    // dxgkrnl VidPn re-negotiation that ChangeDisplaySettingsExW triggers
    // (DxgkDdiEnumVidPnCofuncModality / DxgkDdiCommitVidPn), independent of
    // this flag.
    LONG result = ChangeDisplaySettingsExW(target.deviceName.c_str(), &mode, nullptr, CDS_UPDATEREGISTRY, nullptr);

    std::wcout << L"Requested mode: " << width << L"x" << height << L"\n";
    std::wcout << L"ChangeDisplaySettingsExW returned " << result << L" (" << DispChangeToString(result) << L")\n";

    DEVMODEW current = {};
    current.dmSize = sizeof(current);
    if (EnumDisplaySettingsExW(target.deviceName.c_str(), ENUM_CURRENT_SETTINGS, &current, 0))
    {
        std::wcout << L"Observed mode: " << current.dmPelsWidth << L"x" << current.dmPelsHeight;
        if (current.dmDisplayFrequency != 0)
        {
            std::wcout << L"@" << current.dmDisplayFrequency;
        }
        std::wcout << L"\n";
    }

    return ResultToExitCode(result);
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
