@echo off
rem Startet die Bedienoberflaeche (python -m bc.gui) per Doppelklick.
rem
rem Sucht das Python der Projektumgebung in dieser Reihenfolge:
rem   1. Umgebungsvariable BC_PYTHON (voller Pfad zu python.exe)
rem   2. E:\Enviroments\bc_env\python.exe              (PC, mit torch/lerobot)
rem   3. C:\Enviroments\bc_env312\Scripts\python.exe   (Laptop, Python 3.12)
rem   4. C:\Enviroments\bc_env\Scripts\python.exe      (Laptop, Python 3.11)
rem
rem Weitere Argumente gehen an die GUI, z. B.:  BC-Oberflaeche.bat --mode recording
rem Mit gesetztem BC_GUI_ECHO=1 wird nur angezeigt, was gestartet wuerde.
rem
rem Das Konsolenfenster startet minimiert und zeigt das Log. Beendet sich die
rem GUI mit einem Fehler, bleibt es offen, damit die Meldung lesbar ist.

setlocal
cd /d "%~dp0"

set "PY="
if defined BC_PYTHON if exist "%BC_PYTHON%" set "PY=%BC_PYTHON%"
if not defined PY if exist "E:\Enviroments\bc_env\python.exe" set "PY=E:\Enviroments\bc_env\python.exe"
if not defined PY if exist "C:\Enviroments\bc_env312\Scripts\python.exe" set "PY=C:\Enviroments\bc_env312\Scripts\python.exe"
if not defined PY if exist "C:\Enviroments\bc_env\Scripts\python.exe" set "PY=C:\Enviroments\bc_env\Scripts\python.exe"

if not defined PY (
    echo Kein Python der Projektumgebung gefunden.
    echo Gesucht: BC_PYTHON, E:\Enviroments\bc_env, C:\Enviroments\bc_env312, C:\Enviroments\bc_env
    echo Abhilfe: Umgebungsvariable BC_PYTHON auf den Pfad zu python.exe setzen.
    pause
    exit /b 1
)

rem Conda-Umgebung (PC): DLLs und ffmpeg liegen in Library\bin -- ohne
rem aktivierte Umgebung sonst nicht im PATH.
for %%I in ("%PY%") do set "ENVDIR=%%~dpI"
if exist "%ENVDIR%Library\bin" set "PATH=%ENVDIR%;%ENVDIR%Library\bin;%ENVDIR%Scripts;%PATH%"

if "%BC_GUI_ECHO%"=="1" (
    echo Arbeitsordner: %CD%
    echo Python:        %PY%
    echo Start:         "%PY%" -m bc.gui %*
    exit /b 0
)

start "BC-Oberflaeche (Log)" /min cmd /c ""%PY%" -m bc.gui %* || pause"
endlocal
