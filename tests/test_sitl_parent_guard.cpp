#include "sitl/sitl_platform.h"

#ifdef _WIN32
#include <windows.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

namespace
{
bool spawn_self(const wchar_t* argument, PROCESS_INFORMATION& process)
{
    wchar_t executable[MAX_PATH] = {};
    if (::GetModuleFileNameW(nullptr, executable, MAX_PATH) == 0)
        return false;
    std::wstring command = L"\"";
    command += executable;
    command += L"\" ";
    command += argument;
    STARTUPINFOW startup{};
    startup.cb = sizeof(startup);
    process = {};
    return ::CreateProcessW(
               executable, command.data(), nullptr, nullptr, FALSE,
               CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process) != FALSE;
}

int run_guard_subprocess()
{
    PROCESS_INFORMATION parent{};
    if (!spawn_self(L"--short-lived-parent", parent))
        return 10;
    const DWORD parent_pid = parent.dwProcessId;
    {
        hydrox::sitl::ParentProcessGuard guard(parent_pid);
        if (!guard.arm())
            return 11;
        if (::WaitForSingleObject(parent.hProcess, 5000) != WAIT_OBJECT_0)
            return 12;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::seconds(2);
        while (guard.is_parent_alive() &&
               std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (guard.is_parent_alive())
            return 13;
    } // The guard watchdog must not terminate this process during cleanup.
    ::CloseHandle(parent.hThread);
    ::CloseHandle(parent.hProcess);
    return 47;
}
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string(argv[1]) == "--short-lived-parent")
    {
        ::Sleep(350);
        return 0;
    }
    if (argc == 2 && std::string(argv[1]) == "--guard-subprocess")
        return run_guard_subprocess();
    PROCESS_INFORMATION guard_process{};
    if (!spawn_self(L"--guard-subprocess", guard_process))
        return 1;
    if (::WaitForSingleObject(guard_process.hProcess, 10000) != WAIT_OBJECT_0)
    {
        ::TerminateProcess(guard_process.hProcess, 2);
        ::CloseHandle(guard_process.hThread);
        ::CloseHandle(guard_process.hProcess);
        return 2;
    }
    DWORD exit_code = 0;
    const bool got_code =
        ::GetExitCodeProcess(guard_process.hProcess, &exit_code) != FALSE;
    ::CloseHandle(guard_process.hThread);
    ::CloseHandle(guard_process.hProcess);
    if (!got_code || exit_code != 47)
    {
        std::fprintf(stderr, "guard subprocess exited %lu, expected 47\n",
                     static_cast<unsigned long>(exit_code));
        return 3;
    }
    std::puts("test_sitl_parent_guard passed: parent loss permits cleanup");
    return 0;
}
#else
int main() { return 0; }
#endif
