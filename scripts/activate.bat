@echo off
if exist "%PIXI_PROJECT_ROOT%\install\local_setup.bat" (
    call "%PIXI_PROJECT_ROOT%\install\local_setup.bat"
    if errorlevel 1 exit /b 1
)
