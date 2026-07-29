@echo off

set SYSCFG_PATH="A:\ti\sysconfig_1.26.2\sysconfig_cli.bat"

if not exist "%SYSCFG_PATH%" (
    echo.
    echo Couldn't find Sysconfig Tool %SYSCFG_PATH%
    echo "Update the file located at <sdk path>/tools/keil/syscfg.bat"
    echo.
    exit /b 1
)

echo Using Sysconfig Tool from %SYSCFG_PATH%
echo "Update the file located at <sdk path>/tools/keil/syscfg.bat to use a different version"

set PROJ_DIR=%~1
set PROJ_DIR=%PROJ_DIR:'=%

set SYSCFG_FILE=%~2
set SYSCFG_FILE=%SYSCFG_FILE:'=%

set SDK_ROOT=A:\ti\mspm0_sdk_2_10_00_04
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

%SYSCFG_PATH% -o "%PROJ_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil "%SYSCFG_DIR%\%SYSCFG_FILE%"

:: Patch generated header for ARMCLANG && patch builder.params with missing source files
powershell -ExecutionPolicy Bypass -Command ^
"$hdr = '%PROJ_DIR%\ti_msp_dl_config.h'; ^
 $bp  = '%PROJ_DIR%\build\M0G3519_nortos_keil\builder.params'; ^
 (Get-Content $hdr -Raw).Replace('#elif defined(__GNUC__)', '#elif defined(__GNUC__) || defined(__clang__) || defined(__ARMCC_VERSION)') | Set-Content $hdr -NoNewline; ^
 $j = Get-Content $bp -Raw | ConvertFrom-Json; ^
 $j.sourceList = @( ^
   '../src/MSPM0G3519_Library/zf_common/zf_common_clock.c', ^
   '../src/MSPM0G3519_Library/zf_common/zf_common_debug.c', ^
   '../src/MSPM0G3519_Library/zf_common/zf_common_fifo.c', ^
   '../src/MSPM0G3519_Library/zf_common/zf_common_font.c', ^
   '../src/MSPM0G3519_Library/zf_common/zf_common_interrupt.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_delay.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_encoder.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_gpio.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_pwm.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_timer.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_driver_uart.c', ^
   '../src/MSPM0G3519_Library/zf_driver/zf_sdk_compat.c', ^
   '../src/app/grayscale_app.c', ^
   '../src/app/heartbeat_app.c', ^
   '../src/app/imu_app.c', ^
   '../src/app/motor_app.c', ^
   '../src/app/oled_app.c', ^
   '../src/hardware/encoder_hw.c', ^
   '../src/hardware/grayscale_hw.c', ^
   '../src/hardware/heartbeat_hw.c', ^
   '../src/hardware/imu_hw.c', ^
   '../src/hardware/motor_hw.c', ^
   '../src/hardware/oled_hw.c', ^
   '../src/main.c', ^
   '../src/middle/encoder.c', ^
   '../src/middle/grayscale.c', ^
   '../src/middle/heartbeat.c', ^
   '../src/middle/imu.c', ^
   '../src/middle/motor.c', ^
   '../src/middle/oled.c', ^
   'startup_mspm0g351x_uvision.s', ^
   'ti_msp_dl_config.c' ^
 ); ^
 $j.defines = @('__MSPM0G3519__', 'SYSCONFIG_WEAK=__attribute__((weak))'); ^
 $j | ConvertTo-Json -Depth 10 | Set-Content $bp -NoNewline"
