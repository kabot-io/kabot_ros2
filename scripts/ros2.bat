@echo off
rem Reload the overlay after the build, including on the first invocation.
call "%PIXI_PROJECT_ROOT%\install\local_setup.bat"
if errorlevel 1 exit /b 1
ros2 %*
exit /b %errorlevel%
