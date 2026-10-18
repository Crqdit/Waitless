@echo off
chcp 65001 >nul
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (py -3 podu_sim\podu_sim.py %*) else (python podu_sim\podu_sim.py %*)
if errorlevel 1 pause
