[CmdletBinding()]
param(
    [switch]$Clean,
    [switch]$Offline,
    [string]$BuildDirectory,
    [string]$PythonExecutable,
    [string]$CMakeExecutable,
    [string]$NinjaExecutable,
    [string]$ToolchainRoot
)

$ErrorActionPreference = 'Stop'
$repositoryRoot = [IO.Path]::GetFullPath($PSScriptRoot)
$buildRoot = [IO.Path]::GetFullPath((Join-Path $repositoryRoot 'build'))
if (-not $BuildDirectory) {
    $BuildDirectory = Join-Path $buildRoot 'fmu_v6c_hitl'
}
$buildDirectoryFull = [IO.Path]::GetFullPath($BuildDirectory)
if (-not $buildDirectoryFull.StartsWith($buildRoot + [IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)) {
    throw "BuildDirectory must be a child of $buildRoot"
}

function Find-Program {
    param([string]$Explicit, [string[]]$Names, [string[]]$Fallbacks)
    if ($Explicit) {
        $candidate = [IO.Path]::GetFullPath($Explicit)
        if (-not (Test-Path -LiteralPath $candidate -PathType Leaf)) {
            throw "Required executable was not found: $candidate"
        }
        return $candidate
    }
    foreach ($fallback in $Fallbacks) {
        if (Test-Path -LiteralPath $fallback -PathType Leaf) {
            return $fallback
        }
    }
    foreach ($name in $Names) {
        $command = Get-Command $name -ErrorAction SilentlyContinue
        if ($command) { return $command.Source }
    }
    throw "Required executable was not found: $($Names -join ', ')"
}

if ($Clean -and (Test-Path -LiteralPath $buildDirectoryFull)) {
    Write-Host "[INFO] Removing clean-build directory: $buildDirectoryFull"
    Remove-Item -LiteralPath $buildDirectoryFull -Recurse -Force
}

$nuttx = Join-Path $repositoryRoot 'third_party\nuttx'
$nuttxApps = Join-Path $repositoryRoot 'third_party\nuttx-apps'
if (-not (Test-Path -LiteralPath (Join-Path $nuttx 'CMakeLists.txt')) -or
    -not (Test-Path -LiteralPath (Join-Path $nuttxApps 'CMakeLists.txt'))) {
    if ($Offline) {
        throw 'Pinned NuttX trees are missing and -Offline was specified.'
    }
    & (Join-Path $repositoryRoot 'tools\fetch_nuttx.ps1')
}

$toolchainLock = Get-Content -LiteralPath (
    Join-Path $repositoryRoot 'third_party\arm_gnu_toolchain.lock.json') -Raw | ConvertFrom-Json
if (-not $ToolchainRoot) {
    $ToolchainRoot = Join-Path $repositoryRoot $toolchainLock.destination
}
$toolchainRootFull = [IO.Path]::GetFullPath($ToolchainRoot)
$toolchainBin = Join-Path $toolchainRootFull 'bin'
$gcc = Join-Path $toolchainBin 'arm-none-eabi-gcc.exe'
if (-not (Test-Path -LiteralPath $gcc -PathType Leaf)) {
    if ($Offline) {
        throw 'Pinned Arm GNU Toolchain is missing and -Offline was specified.'
    }
    & (Join-Path $repositoryRoot 'tools\fetch_arm_gnu_toolchain.ps1') `
        -RepositoryRoot $repositoryRoot
}
if (-not (Test-Path -LiteralPath $gcc -PathType Leaf)) {
    throw "Arm GNU compiler was not installed: $gcc"
}

$python = Find-Program -Explicit $PythonExecutable -Names @('python.exe', 'python') `
    -Fallbacks @('C:\Users\Administrator\miniconda3\envs\ros2\python.exe')
$cmake = Find-Program -Explicit $CMakeExecutable -Names @('cmake.exe', 'cmake') `
    -Fallbacks @('C:\Users\Administrator\miniconda3\envs\ros2\Library\bin\cmake.exe')
$ninja = Find-Program -Explicit $NinjaExecutable -Names @('ninja.exe', 'ninja') `
    -Fallbacks @('C:\Users\Administrator\miniconda3\envs\ros2\Library\bin\ninja.exe')

$pythonEnvironment = Join-Path $buildRoot 'fmuv6c-build-python'
$venvPython = Join-Path $pythonEnvironment 'Scripts\python.exe'
if (-not (Test-Path -LiteralPath $venvPython -PathType Leaf)) {
    if ($Offline) {
        throw 'The pinned FMUv6C Python build environment is missing and -Offline was specified.'
    }
    New-Item -ItemType Directory -Force -Path $buildRoot | Out-Null
    Write-Host '[INFO] Creating isolated Python environment for NuttX Kconfig'
    & $python -m venv $pythonEnvironment
    if ($LASTEXITCODE -ne 0) { throw 'Python virtual-environment creation failed.' }
}
$olddefconfig = Join-Path $pythonEnvironment 'Scripts\olddefconfig.exe'
if (-not (Test-Path -LiteralPath $olddefconfig -PathType Leaf)) {
    if ($Offline) {
        throw 'kconfiglib is missing from the build environment and -Offline was specified.'
    }
    Write-Host '[INFO] Installing pinned NuttX Python build dependency'
    & $venvPython -m pip install --disable-pip-version-check `
        -r (Join-Path $repositoryRoot 'requirements-fmuv6c-build.txt')
    if ($LASTEXITCODE -ne 0) { throw 'Installing the pinned Python dependency failed.' }
}

& (Join-Path $repositoryRoot 'tools\prepare_nuttx_external.ps1') `
    -RepositoryRoot $repositoryRoot
if ($LASTEXITCODE -ne 0) { throw 'Preparing the NuttX external application link failed.' }

$oldPath = $env:Path
try {
    $env:Path = @(
        $toolchainBin,
        (Join-Path $pythonEnvironment 'Scripts'),
        (Split-Path -Parent $ninja),
        (Split-Path -Parent $cmake),
        $oldPath
    ) -join [IO.Path]::PathSeparator

    New-Item -ItemType Directory -Force -Path $buildDirectoryFull | Out-Null
    $boardConfig = Join-Path $repositoryRoot 'boards\fmu_v6c\configs\hydrox_hitl'
    Write-Host '[INFO] Configuring Apache NuttX 13 FMUv6C HITL target'
    & $cmake -S $nuttx -B $buildDirectoryFull -G Ninja `
        "-DBOARD_CONFIG=$boardConfig" `
        "-DNUTTX_APPS_DIR=$nuttxApps" `
        "-DHYDROX_SOURCE_DIR=$repositoryRoot" `
        "-DPython3_EXECUTABLE=$venvPython" `
        "-DCMAKE_MAKE_PROGRAM=$ninja"
    if ($LASTEXITCODE -ne 0) { throw 'NuttX CMake configuration failed.' }

    Write-Host '[INFO] Cross-compiling and packaging HydroX FMUv6C HITL firmware'
    & $cmake --build $buildDirectoryFull --target hydrox_pixhawk6cmini_hitl --parallel
    if ($LASTEXITCODE -ne 0) { throw 'FMUv6C firmware build failed.' }

    $firmwareDirectory = Join-Path $buildDirectoryFull 'firmware'
    $manifest = Join-Path $firmwareDirectory 'hydrox_pixhawk6cmini_hitl.manifest.json'
    & $venvPython (Join-Path $repositoryRoot 'tools\verify_fmuv6c_firmware.py') `
        --repository-root $repositoryRoot `
        --build-dir $buildDirectoryFull `
        --firmware-dir $firmwareDirectory `
        --toolchain-bin $toolchainBin `
        --manifest $manifest
    if ($LASTEXITCODE -ne 0) { throw 'FMUv6C firmware release verification failed.' }

    Write-Host "[OK] Verified HITL-only flash candidate: $firmwareDirectory"
    Write-Host '[SAFE] Keep every physical actuator load disconnected during first hardware qualification.'
}
finally {
    $env:Path = $oldPath
}
