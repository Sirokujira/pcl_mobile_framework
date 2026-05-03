@echo off

REM HINT=http://qiita.com/usagi/items/2623145f22faf54b99e0

cd /d%~dp0
:checkMandatoryLevel
for /f "tokens=1 delims=," %%i in ('whoami /groups /FO CSV /NH') do (
    if "%%~i"=="BUILTIN\Administrators" set ADMIN=yes
    if "%%~i"=="Mandatory Label\High Mandatory Level" set ELEVATED=yes
)

if "%ADMIN%" neq "yes" (
    echo This file needs to be executed with administrator authority{not Administrators Group}
   if "%1" neq "/R" goto runas
   goto exit1
)
if "%ELEVATED%" neq "yes" (
    echo This file needs to be executed with administrator authority{The process has not been promoted}
   if "%1" neq "/R" goto runas
   goto exit1
)


:admins
    REM Install GTK+
    REM powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1
    powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1 x86
    powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1 x86_64
    powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1 armeabi
    powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1 armeabi-v7a
    powershell -NoProfile -ExecutionPolicy Unrestricted  .\Install-PointCloudLibrary.ps1 arm64-v8a

    goto exit1

:runas
    REM Re-run as administrator
    powershell -Command Start-Process -Verb runas "%0" -ArgumentList "/R" 

:exit1