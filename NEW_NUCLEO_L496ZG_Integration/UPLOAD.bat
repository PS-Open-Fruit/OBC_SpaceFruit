@echo off
echo Building project...
make

:: Check if make failed (errorlevel not 0)
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ****************************************
    echo * BUILD FAILED! Aborting upload.       *
    echo ****************************************
    echo.
    pause
    exit /b %ERRORLEVEL%
)

echo.
echo Build successful. Uploading firmware...
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program build/NUCLEO_L496ZG_Integration.elf verify reset exit"