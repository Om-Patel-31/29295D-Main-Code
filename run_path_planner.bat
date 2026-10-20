@echo off
title 29295D Path Studio Launcher
echo Launching 29295D Path Studio...
python path_planner.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo An error occurred while launching Path Studio.
    pause
)

