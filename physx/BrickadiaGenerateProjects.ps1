param(
    [Parameter(Mandatory)]
    [ValidateSet('Win64', 'Linux')]
    [string]$Platform,

    [ValidateSet('All', 'Dynamic', 'Static')]
    [string]$Preset = 'All'
)

$ErrorActionPreference = 'Stop'
$PhysXRoot = $PSScriptRoot

$VsWhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path $VsWhere)) {
    Write-Host 'Error: Visual Studio is not installed.'
    exit 1
}

$env:PM_PACKAGES_ROOT = "$PhysXRoot\packages"

# Set up environment before resolving presets
if ($Platform -eq 'Win64') {
    $env:LLVMInstallDir = 'C:/Program Files/LLVM'
    $env:LLVMToolsVersion = '23.1.0'
    $env:LLVMIncludeVersion = '23'

    # Set up VS developer environment so CMake can find cl.exe for Ninja
    $VsInstallPath = & $VsWhere -version '[18.0,19.0)' -products * -property installationPath -nologo | Select-Object -First 1
    if (-not $VsInstallPath) {
        Write-Host 'Error: Visual Studio 2026 not found.'
        exit 1
    }
    Import-Module "$VsInstallPath\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
    Enter-VsDevShell -VsInstallPath $VsInstallPath -DevCmdArguments '-arch=x64 -host_arch=x64' -SkipAutomaticLocation | Out-Null

    # Ensure ninja is on PATH
    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        Write-Host 'Error: Ninja is not installed. Run Scripts\Setup\Install Ninja.bat.'
        exit 1
    }
} elseif ($Platform -eq 'Linux') {
    if (-not (Test-Path "${env:LINUX_MULTIARCH_ROOT}x86_64-unknown-linux-gnu")) {
        Write-Host 'Error: Cross compile toolchain is not installed.'
        exit 1
    }

    if (-not (Get-Command ninja -ErrorAction SilentlyContinue)) {
        Write-Host 'Error: Ninja is not installed. Run Scripts\Setup\Install Ninja.bat.'
        exit 1
    }

    $env:PM_MINGW_PATH = ''
}

# Resolve presets
$Presets = switch ($Platform) {
    'Win64' {
        switch ($Preset) {
            'Dynamic' { @('vc18win64-brickadia-dynamic-ninja') }
            'Static'  { @('clangwin64-brickadia-ninja', 'clangwin64-brickadia-lto-ninja') }
            'All'     { @('vc18win64-brickadia-dynamic-ninja', 'clangwin64-brickadia-ninja', 'clangwin64-brickadia-lto-ninja') }
        }
    }
    'Linux' {
        @('linux-crosscompile-brickadia-ninja', 'linux-crosscompile-brickadia-lto-ninja')
    }
}

Push-Location $PhysXRoot
try {
    foreach ($P in $Presets) {
        Write-Host "Generating $P..."
        & "$PhysXRoot\generate_projects.bat" $P
        if ($LASTEXITCODE -ne 0) { Write-Host 'Aborting script due to error.'; exit 1 }
    }
} finally { Pop-Location }
