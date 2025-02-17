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
set PM_PACKAGES_ROOT=%~dp0packages
set PM_MINGW_PATH=""

:: Generate linux project
call "generate_projects.bat" linux-crosscompile-brickadia

if not %ERRORLEVEL% == 0 (
	echo Aborting script due to error.
	exit /b 1
)
