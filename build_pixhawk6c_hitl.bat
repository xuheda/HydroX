@echo off
setlocal

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build\pixhawk6c_hitl"

if not exist "%SCRIPT_DIR%\boards\pixhawk6c\CMakeLists.txt" (
    echo [ERROR] The in-tree Pixhawk 6C NuttX board port is not implemented.
    echo         See boards\pixhawk6c\README.md for the required board files.
    exit /b 2
)

where cmake.exe >nul 2>nul
if errorlevel 1 (
    echo [ERROR] cmake.exe was not found in PATH.
    exit /b 2
)

where ninja.exe >nul 2>nul
if errorlevel 1 (
    echo [ERROR] ninja.exe was not found in PATH.
    exit /b 2
)

where arm-none-eabi-g++.exe >nul 2>nul
if errorlevel 1 (
    echo [ERROR] arm-none-eabi-g++.exe was not found in PATH.
    exit /b 2
)

cmake.exe ^
    -S "%SCRIPT_DIR%" ^
    -B "%BUILD_DIR%" ^
    -G Ninja ^
    -DCMAKE_BUILD_TYPE=Release ^
    -DCMAKE_TOOLCHAIN_FILE="%SCRIPT_DIR%\cmake\toolchains\arm_none_eabi_gcc.cmake" ^
    -DHYDROX_TARGET=PIXHAWK6C_HITL
if errorlevel 1 exit /b 1

cmake.exe --build "%BUILD_DIR%" --target hydrox_pixhawk6c_hitl
if errorlevel 1 exit /b 1

set "FIRMWARE=%BUILD_DIR%\firmware\hydrox_pixhawk6c_hitl.bin"
if not exist "%FIRMWARE%" (
    echo [ERROR] Expected firmware was not generated: %FIRMWARE%
    exit /b 1
)

echo [OK] HydroX Pixhawk 6C HITL firmware: %FIRMWARE%
endlocal
exit /b 0
