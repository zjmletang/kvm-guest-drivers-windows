@echo off
call ..\build\build.bat stdvga.vcxproj "Win10 Win11" %*
if errorlevel 1 exit /b 1
call ..\build\build.bat tools\stdvgares\stdvgares.vcxproj "Win10 Win11" %*
