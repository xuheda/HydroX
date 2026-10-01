@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_fmuv6c_hitl.ps1" %*
set "RESULT=%ERRORLEVEL%"
endlocal & exit /b %RESULT%
