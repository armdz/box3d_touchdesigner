@echo off
rem Copies the built Box3D plugin DLLs into the TouchDesigner global plugin folder.
rem Build first with:
rem   cmake --build build --config Release

setlocal
set "SRC=%~dp0plugin"
set "DEST=%USERPROFILE%\Documents\Derivative\Plugins"

if not exist "%SRC%\Box3DSolverCHOP.dll" (
    echo [ERROR] No se encontro "%SRC%\Box3DSolverCHOP.dll".
    echo Compila primero:  cmake --build build --config Release
    exit /b 1
)

if not exist "%DEST%" mkdir "%DEST%"

copy /Y "%SRC%\*.dll" "%DEST%" >nul
if errorlevel 1 (
    echo [ERROR] No se pudo copiar. Si TouchDesigner esta abierto, cerralo e intenta de nuevo
    echo         ^(Windows bloquea la DLL mientras TD la tiene cargada^).
    exit /b 1
)

echo [OK] Plugins copiados a: %DEST%
dir /b "%DEST%\*.dll"
endlocal
