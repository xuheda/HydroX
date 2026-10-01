[CmdletBinding()]
param(
    [string]$RepositoryRoot = (Split-Path -Parent $PSScriptRoot),
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath($RepositoryRoot)
$lockPath = Join-Path $root 'third_party\arm_gnu_toolchain.lock.json'
$lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json
$destination = [IO.Path]::GetFullPath((Join-Path $root $lock.destination))
$toolchainsRoot = [IO.Path]::GetFullPath((Join-Path $root 'third_party\toolchains'))
$compiler = Join-Path $destination 'bin\arm-none-eabi-gcc.exe'

if (-not $destination.StartsWith($toolchainsRoot + [IO.Path]::DirectorySeparatorChar,
        [StringComparison]::OrdinalIgnoreCase)) {
    throw "Toolchain destination escapes the managed toolchains directory: $destination"
}

if ((Test-Path -LiteralPath $compiler -PathType Leaf) -and -not $Force) {
    Write-Host "[OK] Arm GNU Toolchain already installed: $destination"
    Write-Output $destination
    exit 0
}

if ($Force -and (Test-Path -LiteralPath $destination)) {
    Remove-Item -LiteralPath $destination -Recurse -Force
}

New-Item -ItemType Directory -Force -Path $toolchainsRoot | Out-Null
$temporary = Join-Path $toolchainsRoot ('.arm-gnu-' + [guid]::NewGuid().ToString('N'))
$archive = Join-Path $temporary 'toolchain.zip'
$extract = Join-Path $temporary 'extract'
New-Item -ItemType Directory -Path $extract -Force | Out-Null

try {
    Write-Host "[INFO] Downloading Arm GNU Toolchain $($lock.version)"
    $curl = Get-Command curl.exe -ErrorAction SilentlyContinue
    if ($curl) {
        & $curl.Source --fail --location --retry 5 --retry-all-errors `
            --progress-bar --output $archive $lock.url
        if ($LASTEXITCODE -ne 0) {
            throw "curl failed to download Arm GNU Toolchain (exit $LASTEXITCODE)"
        }
    }
    else {
        Invoke-WebRequest -UseBasicParsing -Uri $lock.url -OutFile $archive
    }
    $actualHash = (Get-FileHash -Algorithm SHA256 -LiteralPath $archive).Hash.ToLowerInvariant()
    if ($actualHash -ne $lock.sha256.ToLowerInvariant()) {
        throw "SHA-256 mismatch for Arm GNU Toolchain: expected $($lock.sha256), got $actualHash"
    }

    Expand-Archive -LiteralPath $archive -DestinationPath $extract
    $archiveRoot = Join-Path $extract $lock.archive_root
    if (-not (Test-Path -LiteralPath $archiveRoot -PathType Container)) {
        throw "Expected toolchain archive root was not found: $archiveRoot"
    }
    if (Test-Path -LiteralPath $destination) {
        throw "Toolchain destination already exists: $destination"
    }
    Move-Item -LiteralPath $archiveRoot -Destination $destination

    if (-not (Test-Path -LiteralPath $compiler -PathType Leaf)) {
        throw "Installed toolchain has no compiler: $compiler"
    }
    Write-Host "[OK] Verified Arm GNU Toolchain -> $destination"
    Write-Output $destination
}
finally {
    if (Test-Path -LiteralPath $temporary) {
        Remove-Item -LiteralPath $temporary -Recurse -Force -ErrorAction SilentlyContinue
    }
}
