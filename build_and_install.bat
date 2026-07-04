@echo off
setlocal EnableExtensions

rem Build + install helper for TouchDesigner testing.
rem 1) Build Release
rem 2) Copy plugin DLLs using install_plugin.bat

set "ROOT=%~dp0"
pushd "%ROOT%" >nul

echo [1/2] Compilando Release...
cmake --build build --config Release
if errorlevel 1 (
    echo [ERROR] Fallo la compilacion Release.
    popd >nul
    exit /b 1
)

echo [2/2] Copiando DLLs a TouchDesigner Plugins...
call "%ROOT%install_plugin.bat"
if errorlevel 1 (
    echo [ERROR] Fallo la copia de DLLs. Cierra TouchDesigner y vuelve a intentar.
    popd >nul
    exit /b 1
)

echo [OK] Build + install completado.
popd >nul
exit /b 0
