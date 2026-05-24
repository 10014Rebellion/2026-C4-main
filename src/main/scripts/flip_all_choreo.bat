@echo off

REM Get directory of this script (scripts folder)
set SCRIPT_DIR=%~dp0

REM Go to scripts directory (safe)
cd /d "%SCRIPT_DIR%"

REM Relative path to choreo folder
set CHOREO_DIR=..\deploy\choreo

REM Run the flipper script
python ChoreoPathFlipper.py all "%CHOREO_DIR%"

echo.
echo Done flipping all paths.
pause