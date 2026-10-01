[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$lockPath = Join-Path $projectRoot 'third_party\nuttx.lock.json'
$lock = Get-Content -LiteralPath $lockPath -Raw | ConvertFrom-Json

$tempBase = [System.IO.Path]::GetTempPath()
$tempDirectory = Join-Path $tempBase ("hydrox-nuttx-" + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $tempDirectory | Out-Null

try {
    foreach ($package in $lock.packages) {
        $destination = Join-Path $projectRoot $package.destination
        if (Test-Path -LiteralPath $destination) {
            throw "Destination already exists; remove it explicitly before retrying: $destination"
        }

        $archivePath = Join-Path $tempDirectory ($package.name + '.tar.gz')
        $extractPath = Join-Path $tempDirectory ('extract-' + $package.name)
        New-Item -ItemType Directory -Path $extractPath | Out-Null

        Write-Host "[INFO] Downloading $($package.name) $($lock.version)"
        Invoke-WebRequest -UseBasicParsing -Uri $package.url -OutFile $archivePath

        $actualHash = (Get-FileHash -Algorithm SHA512 -LiteralPath $archivePath).Hash.ToLowerInvariant()
        if ($actualHash -ne $package.sha512.ToLowerInvariant()) {
            throw "SHA-512 mismatch for $($package.name)"
        }

        & tar.exe -xzf $archivePath -C $extractPath
        if ($LASTEXITCODE -ne 0) {
            throw "Failed to extract $($package.name)"
        }

        $archiveRoot = Join-Path $extractPath $package.archive_root
        if (-not (Test-Path -LiteralPath $archiveRoot -PathType Container)) {
            $topLevelDirectories = @(Get-ChildItem -LiteralPath $extractPath -Directory)
            if ($topLevelDirectories.Count -ne 1) {
                $entries = (Get-ChildItem -LiteralPath $extractPath | Select-Object -ExpandProperty Name) -join ', '
                throw "Expected archive root '$($package.archive_root)' was not found and the archive did not contain exactly one root directory: $entries"
            }
            $archiveRoot = $topLevelDirectories[0].FullName
            Write-Warning "Archive root is '$($topLevelDirectories[0].Name)', not '$($package.archive_root)'; using the only extracted root."
        }

        $destinationParent = Split-Path -Parent $destination
        New-Item -ItemType Directory -Force -Path $destinationParent | Out-Null
        Move-Item -LiteralPath $archiveRoot -Destination $destination
        Write-Host "[OK] $($package.name) -> $destination"
    }
}
finally {
    $resolvedTempBase = [System.IO.Path]::GetFullPath($tempBase)
    $resolvedTempDirectory = [System.IO.Path]::GetFullPath($tempDirectory)
    if ($resolvedTempDirectory.StartsWith($resolvedTempBase, [System.StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path -Leaf $resolvedTempDirectory).StartsWith('hydrox-nuttx-')) {
        Remove-Item -LiteralPath $resolvedTempDirectory -Recurse -Force -ErrorAction SilentlyContinue
    }
}
