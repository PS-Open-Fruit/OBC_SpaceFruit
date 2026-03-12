@echo off
SET ELF=build/hardware-test.elf

:: Run Make
make
if %errorlevel% neq 0 (
    echo Build failed.
    exit /b %errorlevel%
)

:: Run OpenOCD
openocd -f interface/stlink.cfg -f target/stm32l4x.cfg -c "program %ELF% verify reset exit"
if %errorlevel% neq 0 (
    echo Programming failed.
    exit /b %errorlevel%
)

:: Display Size Info
echo.
echo Memory Usage:
arm-none-eabi-size --format=berkeley %ELF%