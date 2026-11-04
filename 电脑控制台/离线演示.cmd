@echo off
cd /d "%~dp0"
if exist "D:\Program Files\Python313\pythonw.exe" (
 start "" "D:\Program Files\Python313\pythonw.exe" "%~dp0fpga_console.py" --demo
 exit /b
)
where pythonw >nul 2>nul
if not errorlevel 1 (
 start "" pythonw "%~dp0fpga_console.py" --demo
 exit /b
)
py -3 "%~dp0fpga_console.py" --demo
if errorlevel 1 pause
