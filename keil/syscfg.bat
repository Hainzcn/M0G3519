@echo off
setlocal

set "SYSCFG_CLI=A:\ti\sysconfig_1.26.2\sysconfig_cli.bat"

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

:: Search for the directory containing the project's syscfg file
:: Going up a directory atleast 5 times but then give up
set SYSCFG_DIR=%PROJ_DIR%
set iter=0
:syscfg_search_loop
if exist "%SYSCFG_DIR%\*.syscfg" goto syscfg_found
if %iter% geq 5 goto syscfg_not_found
set /a iter=%iter%+1
set SYSCFG_DIR=%SYSCFG_DIR%..\
goto syscfg_search_loop

:syscfg_not_found
@echo "Couldn't find syscfg file"
exit /b 1

:syscfg_found
:: Remove the trailing slash if it exist since Keil doesn't like it
IF %SYSCFG_DIR:~-1%==\ SET SYSCFG_DIR=%SYSCFG_DIR:~0,-1%
:syscfg_search_exit

"%SYSCFG_CLI%" -o "%PROJ_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil --device MSPM0G3519 --package "LQFP-100(PZ)" "%SYSCFG_DIR%\%SYSCFG_FILE%"

:: Patch generated header for ARMCLANG && add SYSCONFIG_WEAK define to builder.params
powershell -ExecutionPolicy Bypass -Command ^
"$hdr = '%PROJ_DIR%\ti_msp_dl_config.h'; ^
 $bp  = '%PROJ_DIR%\build\M0G3519_nortos_keil\builder.params'; ^
 (Get-Content $hdr -Raw).Replace('#elif defined(__GNUC__)', '#elif defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)') | Set-Content $hdr -NoNewline; ^
 $j = Get-Content $bp -Raw | ConvertFrom-Json; ^
 $hasWeak = $j.defines | Where-Object { $_ -like 'SYSCONFIG_WEAK=*' }; ^
 if (-not $hasWeak) { $j.defines = @($j.defines) + 'SYSCONFIG_WEAK=__attribute__((weak))' }; ^
 $j | ConvertTo-Json -Depth 10 | Set-Content $bp -NoNewline"
