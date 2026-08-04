@echo off
:: ============================================================
::  Build script for hydrox_sitl
::  Visual Studio 2022 + CMake (conda ros2_humble env)
:: ============================================================

setlocal

set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "BUILD_DIR=%SCRIPT_DIR%\build\sitl"
set "BUILD_TARGET=hydrox_sitl"
if not "%~1"=="" set "BUILD_TARGET=%~1"

:: Optional: deploy the built executable to an external runtime directory.
::   build_sitl.bat [target] [runtime_dir]
set "RUNTIME_DIR=%~2"

:: Prefer the ROS conda CMake so one build tree never mixes CMake module versions.
if defined CONDA_PREFIX if exist "%CONDA_PREFIX%\Library\bin\cmake.exe" (
    set "CMAKE=%CONDA_PREFIX%\Library\bin\cmake.exe"
)
if not defined CMAKE if exist "C:\ProgramData\miniconda3\envs\ros2_humble\Library\bin\cmake.exe" (
    set "CMAKE=C:\ProgramData\miniconda3\envs\ros2_humble\Library\bin\cmake.exe"
)
if not defined CMAKE if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
    set "CMAKE=C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
)
if not defined CMAKE if exist "C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe" (
    set "CMAKE=C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin\cmake.exe"
)
if not defined CMAKE (
    set "CMAKE=cmake.exe"
)

:: ---- Configure ----
echo [INFO] Running CMake configure...
if not exist "%BUILD_DIR%" mkdir "%BUILD_DIR%"
"%CMAKE%" -S "%SCRIPT_DIR%" -B "%BUILD_DIR%" -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 (
    echo [ERROR] CMake configure failed.
    exit /b 1
)

:: ---- Build ----
echo.
echo [INFO] Building %BUILD_TARGET% (Release / x64) ...
echo.

"%CMAKE%" --build "%BUILD_DIR%" --config Release --target %BUILD_TARGET% -- /m /nologo /clp:Summary

if errorlevel 1 (
    echo.
    echo [ERROR] Build failed. See output above.
    exit /b 1
)

set "BUILT_EXE=%BUILD_DIR%\Release\hydrox_sitl.exe"
if /I "%BUILD_TARGET%"=="hydrox_sitl" (
    if not exist "%BUILT_EXE%" (
        echo.
        echo [ERROR] Built HydroX runtime not found: %BUILT_EXE%
        exit /b 1
    )
    if defined RUNTIME_DIR (
        if not exist "%RUNTIME_DIR%\" mkdir "%RUNTIME_DIR%"
        if errorlevel 1 (
            echo.
            echo [ERROR] Failed to create runtime directory: %RUNTIME_DIR%
            exit /b 1
        )
        copy /Y "%BUILT_EXE%" "%RUNTIME_DIR%\hydrox_sitl.exe" >nul
        if errorlevel 1 (
            echo.
            echo [ERROR] Failed to deploy HydroX runtime: %BUILT_EXE%
            exit /b 1
        )
        echo      Runtime: %RUNTIME_DIR%\hydrox_sitl.exe
    )
)

echo.
echo [OK] Build succeeded.
echo      Target: %BUILD_TARGET%
echo      Output directory: %BUILD_DIR%\Release
echo.
endlocal
exit /b 0
