@echo off
rem Drag a CCS-built .out file onto this file, or double-click it to choose one.
rem Press W in the window to switch worlds.
cd /d "%~dp0"
python -m rslk_sim %* --world line_oval
if errorlevel 1 pause
