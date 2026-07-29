@echo off
setlocal

set "SYSCFG_CLI=A:\ti\sysconfig_1.28.0\sysconfig_cli.bat"

if not exist "%SYSCFG_CLI%" (
    echo.
    echo Couldn't find Sysconfig Tool "%SYSCFG_CLI%"
    echo "Update the file located at <sdk path>/tools/keil/syscfg.bat"
    echo.
    exit /b 1
)

echo Using Sysconfig Tool from "%SYSCFG_CLI%"
echo "Update the file located at <sdk path>/tools/keil/syscfg.bat to use a different version"

set "PROJ_DIR=%~1"
set "PROJ_DIR=%PROJ_DIR:'=%"
if "%PROJ_DIR:~-1%"=="\" set "PROJ_DIR=%PROJ_DIR:~0,-1%"
set "SYSCFG_FILE=%~2"
set "SYSCFG_FILE=%SYSCFG_FILE:'=%"

set "SDK_ROOT=A:\ti\mspm0_sdk_2_10_00_04"
if not exist "%SDK_ROOT%\.metadata\product.json" (
    echo.
    echo Couldn't find SDK metadata at %SDK_ROOT%\.metadata\product.json
    echo Update SDK_ROOT in syscfg.bat to your local MSPM0 SDK path.
    echo.
    exit /b 1
)

for %%I in ("%PROJ_DIR%\..\%SYSCFG_FILE%") do set "SYSCFG_FULL=%%~fI"
if not exist "%SYSCFG_FULL%" (
    echo Couldn't find SysConfig file "%SYSCFG_FULL%"
    exit /b 1
)

"%SYSCFG_CLI%" -o "%PROJ_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil --device MSPM0G3519 --package "LQFP-100(PZ)" "%SYSCFG_FULL%"
