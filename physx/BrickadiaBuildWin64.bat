@echo off

:: Check VS installer
if not exist "C:\Program Files (x86)\Microsoft Visual Studio\Installer" (
	echo Error: Visual Studio is not installed.
	exit /b 1
)

:: Locate MSBuild
pushd "C:\Program Files (x86)\Microsoft Visual Studio\Installer"
for /f "usebackq tokens=* delims=" %%i in (`vswhere.exe -version "[17.0,18.0)" -products * -requires Microsoft.Component.MSBuild -property installationPath -nologo`) do (
	set VisualStudioLocation=%%i
)
popd

if not exist "%VisualStudioLocation%" (
	echo Error: Visual Studio is not installed.
	exit /b 1
)

:: Prepare
WHERE msbuild

if not %ERRORLEVEL% == 0 (
	call "%VisualStudioLocation%\VC\Auxiliary\Build\vcvars64.bat"
)

set PM_PACKAGES_ROOT=%~dp0packages

set LLVMInstallDir=C:/Program Files/LLVM
set LLVMToolsVersion=19.1.3
set LLVMIncludeVersion=19

set BuildDebug=0
set BuildChecked=0
set BuildProfile=0
set BuildRelease=0

if "%1"=="" (
	set BuildDebug=1
	set BuildChecked=1
	set BuildProfile=1
	set BuildRelease=1
)

if "%1"=="debug" set BuildDebug=1
if "%1"=="checked" set BuildChecked=1
if "%1"=="profile" set BuildProfile=1
if "%1"=="release" set BuildRelease=1

:: Compile targets
if "%BuildDebug%"=="1" (
	echo Building debug...
	msbuild "%~dp0compiler\vc17win64-brickadia-dynamic\PhysXSDK.sln" /m /property:Configuration=debug
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildChecked%"=="1" (
	echo Building checked...
	msbuild "%~dp0compiler\vc17win64-brickadia-dynamic\PhysXSDK.sln" /m /property:Configuration=checked
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildProfile%"=="1" (
	echo Building profile...
	msbuild "%~dp0compiler\vc17win64-brickadia-dynamic\PhysXSDK.sln" /m /property:Configuration=profile
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildRelease%"=="1" (
	echo Building release...
	msbuild "%~dp0compiler\clangwin64-brickadia\PhysXSDK.sln" /m /property:Configuration=release
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)
