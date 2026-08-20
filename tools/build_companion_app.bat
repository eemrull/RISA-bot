@echo off
title Build RISA-Bot Companion App Release
setlocal

echo [INFO] Publishing RisaBotApp to dist\RisaBotApp...
dotnet publish "%~dp0..\RisaBotApp\RisaBotApp.csproj" -f net10.0-windows10.0.19041.0 -c Release -p:WindowsPackageType=None -o "%~dp0..\dist\RisaBotApp"

if %ERRORLEVEL% equ 0 (
    echo [SUCCESS] Release build ready at dist\RisaBotApp\RisaBotApp.exe
) else (
    echo [FAILED] Build failed with error %ERRORLEVEL%
)
pause
