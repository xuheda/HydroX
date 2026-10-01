param(
    [Parameter(Mandatory = $true)]
    [string]$RepositoryRoot
)

$ErrorActionPreference = 'Stop'
$root = [IO.Path]::GetFullPath($RepositoryRoot)
$apps = Join-Path $root 'third_party\nuttx-apps'
$source = Join-Path $root 'nuttx_apps\external'
$link = Join-Path $apps 'external'

if (-not (Test-Path -LiteralPath (Join-Path $apps 'CMakeLists.txt'))) {
    throw "Pinned NuttX apps tree is missing: $apps"
}
if (-not (Test-Path -LiteralPath (Join-Path $source 'CMakeLists.txt'))) {
    throw "HydroX external NuttX applications are missing: $source"
}

if (Test-Path -LiteralPath $link) {
    $item = Get-Item -LiteralPath $link -Force
    $target = @($item.Target)[0]
    if ($null -eq $target) {
        throw "Refusing to replace existing non-link directory: $link"
    }
    $resolvedTarget = [IO.Path]::GetFullPath(
        $(if ([IO.Path]::IsPathRooted($target)) { $target } else { Join-Path $apps $target }))
    if (-not $resolvedTarget.Equals([IO.Path]::GetFullPath($source), [StringComparison]::OrdinalIgnoreCase)) {
        throw "Existing NuttX external link points elsewhere: $resolvedTarget"
    }
    exit 0
}

New-Item -ItemType Junction -Path $link -Target $source | Out-Null
Write-Host "[OK] Linked NuttX apps external -> $source"