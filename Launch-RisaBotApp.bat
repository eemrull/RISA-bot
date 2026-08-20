@echo off
title RISA-Bot Companion App
setlocal

set "APP_EXE=%~dp0dist\RisaBotApp\RisaBotApp.exe"

if not exist "%APP_EXE%" (
    echo [INFO] Prebuilt binary not found in dist\. Building RisaBotApp...
    dotnet publish "%~dp0RisaBotApp\RisaBotApp.csproj" -f net10.0-windows10.0.19041.0 -c Release -p:WindowsPackageType=None -o "%~dp0dist\RisaBotApp"
)

if exist "%APP_EXE%" (
    start "" "%APP_EXE%"
) else (
    echo [ERROR] Could not start RisaBotApp.exe.
    pause
)
