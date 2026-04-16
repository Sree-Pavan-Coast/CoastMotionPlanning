[CmdletBinding()]
param(
    [ValidateSet("Debug", "Release")]
    [string]$Configuration = "Release",
    [switch]$Clean,
    [switch]$BuildTests
)

$ErrorActionPreference = "Stop"

# Resolve paths
$repoRoot = Split-Path -Parent $PSScriptRoot
$buildDir = Join-Path $repoRoot "build"
$toolchain = "D:\vcpkg\scripts\buildsystems\vcpkg.cmake"
$vcpkgRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $toolchain))
$boostRoot = Join-Path $vcpkgRoot "installed\x64-windows"
$eigen3Dir = Join-Path $boostRoot "share\eigen3"

# Optional clean
if ($Clean -and (Test-Path $buildDir)) {
    Remove-Item -Recurse -Force $buildDir
}

if (-not (Test-Path $buildDir)) {
    New-Item -ItemType Directory -Path $buildDir | Out-Null
}

Push-Location $buildDir
try {
    $buildTestsFlag = if ($BuildTests.IsPresent) { "ON" } else { "OFF" }
    $cmakeArgs = @(
        "-DCMAKE_BUILD_TYPE=$Configuration",
        "-DCMAKE_TOOLCHAIN_FILE=$toolchain",
        "-DCMAKE_PREFIX_PATH=$boostRoot",
        "-DBOOST_ROOT=$boostRoot",
        "-DBoost_INCLUDE_DIR=$boostRoot\include",
        "-DEigen3_DIR=$eigen3Dir",
        "-DBUILD_TESTS=$buildTestsFlag",
        ".."
    )
    cmake @cmakeArgs
    cmake --build . --config $Configuration
}
finally {
    Pop-Location
}

