@echo off
rem Startet die GUI der Gruppe (code/) mit dem Reiter "Behavior Cloning".
rem Nur im Repo ProjektMaster_DynamicPicking, mit diesem Ordner neben code/
rem (siehe INTEGRATION.md). Weitere Argumente gehen an die GUI, z. B. --tab m2
rem
rem Zwei Umgebungen, bewusst getrennt:
rem   GUI          ..\code\.venv\Scripts\python.exe   (die der Gruppe)
rem   BC_PYTHON    Python der BC-Umgebung fuer Training, Export, Aufnahme.
rem                Gesucht: BC_PYTHON, .venv\ in diesem Ordner,
rem                E:\Enviroments\bc_env, C:\Enviroments\bc_env312
rem
rem Mit gesetztem BC_GUI_ECHO=1 wird nur angezeigt, was gestartet wuerde.

setlocal
cd /d "%~dp0"

set "GUI_PY=%~dp0..\code\.venv\Scripts\python.exe"
if not exist "%GUI_PY%" (
    echo Python der Gruppen-GUI nicht gefunden: %GUI_PY%
    echo Abhilfe: venv in code\ einrichten, siehe deren ANLEITUNG.md Abschnitt 2.
    pause
    exit /b 1
)

set "PY="
if defined BC_PYTHON if exist "%BC_PYTHON%" set "PY=%BC_PYTHON%"
if not defined PY if exist "%~dp0.venv\Scripts\python.exe" set "PY=%~dp0.venv\Scripts\python.exe"
if not defined PY if exist "E:\Enviroments\bc_env\python.exe" set "PY=E:\Enviroments\bc_env\python.exe"
if not defined PY if exist "C:\Enviroments\bc_env312\Scripts\python.exe" set "PY=C:\Enviroments\bc_env312\Scripts\python.exe"
if not defined PY (
    echo Kein Python der BC-Umgebung gefunden - Training und Aufnahme gehen dann nicht.
    echo Abhilfe: Umgebungsvariable BC_PYTHON auf den Pfad zu python.exe setzen.
    pause
    exit /b 1
)
set "BC_PYTHON=%PY%"

if "%BC_GUI_ECHO%"=="1" (
    echo GUI:       "%GUI_PY%" start_gruppen_gui.py %*
    echo BC_PYTHON: %BC_PYTHON%
    exit /b 0
)

start "Gruppen-GUI mit BC (Log)" /min cmd /c ""%GUI_PY%" start_gruppen_gui.py %* || pause"
endlocal
