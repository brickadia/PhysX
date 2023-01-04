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
set PATH=%PATH%;"%BRICKADIA_UNREAL_DIR%\Engine\Extras\ThirdPartyNotUE\GNU_Make\make-3.81\bin"

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
	pushd "%~dp0compiler\linux-crosscompile-brickadia-debug\"
	make -j 24 "MAKE=make -j 24"
	popd
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildChecked%"=="1" (
	echo Building checked...
	pushd "%~dp0compiler\linux-crosscompile-brickadia-checked\"
	make -j 24 "MAKE=make -j 24"
	popd
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildProfile%"=="1" (
	echo Building profile...
	pushd "%~dp0compiler\linux-crosscompile-brickadia-profile\"
	make -j 24 "MAKE=make -j 24"
	popd
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)

if "%BuildRelease%"=="1" (
	echo Building release...
	pushd "%~dp0compiler\linux-crosscompile-brickadia-release\"
	make -j 24 "MAKE=make -j 24"
	popd
	if not %ERRORLEVEL% == 0 (
		echo Aborting script due to error.
		exit /b 1
	)
)
