param(
    [Parameter(Mandatory)]
    [ValidateSet('Win64', 'Linux')]
    [string]$Platform,

    [ValidateSet('debug', 'checked', 'profile', 'release')]
    [string[]]$Configs = @('debug', 'checked', 'release')
)

$ErrorActionPreference = 'Stop'
$PhysXRoot = $PSScriptRoot

function Invoke-Build([string]$Name, [scriptblock]$Action) {
    Write-Host "Building $Name..."
    & $Action 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Host 'Aborting script due to error.'; exit 1 }
}

switch ($Platform) {
    'Win64' {
        # Find Visual Studio installation
        $VsWhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
        if (-not (Test-Path $VsWhere)) {
            Write-Host 'Error: Visual Studio is not installed.'
            exit 1
        }

        $VsInstallPath = & $VsWhere -version '[18.0,19.0)' -products * -property installationPath -nologo | Select-Object -First 1
        if (-not $VsInstallPath) {
            Write-Host 'Error: Visual Studio 2026 not found.'
            exit 1
        }

        # Set up VS developer environment for cl.exe
        $DevShellModule = "$VsInstallPath\Common7\Tools\Microsoft.VisualStudio.DevShell.dll"
        if (-not (Test-Path $DevShellModule)) {
            Write-Host 'Error: VS DevShell module not found.'
            exit 1
        }
        Import-Module $DevShellModule
        Enter-VsDevShell -VsInstallPath $VsInstallPath -DevCmdArguments '-arch=x64 -host_arch=x64' -SkipAutomaticLocation

        # LLVM for Clang builds
        $env:LLVMInstallDir = 'C:/Program Files/LLVM'
        $env:LLVMToolsVersion = '22.1.6'
        $env:LLVMIncludeVersion = '22'

        $DynamicConfigs = @($Configs | Where-Object { $_ -in @('debug', 'checked', 'profile') })
        if ($DynamicConfigs.Count -gt 0) {
            $ConfigNames = $DynamicConfigs -join ', '
            $Targets = ($DynamicConfigs | ForEach-Object { "all:$_" }) -join ' '
            Invoke-Build "MSVC dynamic ($ConfigNames)" {
                ninja -C "$PhysXRoot\compiler\vc18win64-brickadia-dynamic-ninja" $Targets.Split(' ')
            }
        }

        if ('release' -in $Configs) {
            Invoke-Build 'Clang static (release)' {
                ninja -C "$PhysXRoot\compiler\clangwin64-brickadia-ninja" all:release
            }
            Invoke-Build 'Clang static LTO (release-lto)' {
                ninja -C "$PhysXRoot\compiler\clangwin64-brickadia-lto-ninja" all:release
            }
        }
    }
    'Linux' {
        if (-not (Test-Path "${env:LINUX_MULTIARCH_ROOT}x86_64-unknown-linux-gnu")) {
            Write-Host 'Error: Cross compile toolchain is not installed.'
            exit 1
        }

        $env:PM_PACKAGES_ROOT = "$PhysXRoot\packages"

        $StandardConfigs = $Configs | Where-Object { $_ -ne 'release' }
        foreach ($Config in $StandardConfigs) {
            Invoke-Build $Config {
                cmake --build "$PhysXRoot\compiler\linux-crosscompile-brickadia-ninja-$Config"
            }
        }

        if ('release' -in $Configs) {
            Invoke-Build 'release' {
                cmake --build "$PhysXRoot\compiler\linux-crosscompile-brickadia-ninja-release"
            }
            Invoke-Build 'release-lto' {
                cmake --build "$PhysXRoot\compiler\linux-crosscompile-brickadia-lto-ninja-release"
            }
        }
    }
}
