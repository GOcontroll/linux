@echo off
REM inject-modules.bat — Windows wrapper voor inject-modules.sh via WSL.
REM
REM ext4-mount + e2fsprogs (resize2fs/e2fsck/tune2fs/depmod) zijn Linux-only,
REM dus we delegeren naar WSL. De gebruiker hoeft niets van Linux te weten;
REM dit script vertaalt Windows-paden naar /mnt/c/... en roept de bash-versie
REM aan met sudo.
REM
REM Vereist:
REM   - WSL2 + een Debian/Ubuntu distro met e2fsprogs + rsync + kmod
REM     (apt install e2fsprogs rsync kmod)
REM   - inject-modules.sh in dezelfde map als deze .bat
REM   - sudo zonder wachtwoord OF user typt wachtwoord in PowerShell-window
REM
REM Gebruik:
REM   inject-modules.bat                                 (defaults: rootfs13_headless.ext4 + .\modules)
REM   inject-modules.bat C:\path\rootfs.ext4 C:\path\modules

setlocal enabledelayedexpansion

REM --- defaults: zoek rootfs + modules naast het script zelf ---
set "SCRIPT_DIR=%~dp0"
set "ROOTFS=%~1"
if "%ROOTFS%"=="" set "ROOTFS=%SCRIPT_DIR%rootfs13_headless.ext4"
set "MODULES=%~2"
if "%MODULES%"=="" set "MODULES=%SCRIPT_DIR%modules"

REM --- check beschikbaarheid ---
if not exist "%ROOTFS%" (
    echo ERROR: rootfs niet gevonden: %ROOTFS%
    echo Geef pad als arg 1, of zet rootfs13_headless.ext4 naast deze .bat.
    exit /b 1
)
if not exist "%MODULES%" (
    echo ERROR: modules-dir niet gevonden: %MODULES%
    echo Geef pad als arg 2, of zet modules\ naast deze .bat.
    exit /b 1
)
if not exist "%SCRIPT_DIR%inject-modules.sh" (
    echo ERROR: inject-modules.sh ontbreekt naast deze .bat
    echo Verwacht in %SCRIPT_DIR%
    exit /b 1
)

REM --- WSL aanwezig? ---
where wsl >nul 2>&1 || (
    echo ERROR: WSL niet aanwezig op deze machine.
    echo Installeer WSL2 ^(wsl --install^) of run inject-modules.sh direct in een Linux/WSL omgeving.
    exit /b 2
)

REM --- vertaal Windows-paden naar WSL-paden ---
for /f "usebackq delims=" %%i in (`wsl wslpath -a "%ROOTFS%"`)                  do set "ROOTFS_WSL=%%i"
for /f "usebackq delims=" %%i in (`wsl wslpath -a "%MODULES%"`)                 do set "MODULES_WSL=%%i"
for /f "usebackq delims=" %%i in (`wsl wslpath -a "%SCRIPT_DIR%inject-modules.sh"`) do set "SCRIPT_WSL=%%i"

echo [inject-modules.bat] rootfs:  %ROOTFS_WSL%
echo [inject-modules.bat] modules: %MODULES_WSL%
echo [inject-modules.bat] script:  %SCRIPT_WSL%
echo.

REM --- run via WSL met sudo ---
REM (sudo prompt verschijnt in dit cmd-window; user wachtwoord is van de WSL-user)
wsl sudo bash "%SCRIPT_WSL%" "%ROOTFS_WSL%" "%MODULES_WSL%"
set RC=%ERRORLEVEL%

if %RC% neq 0 (
    echo.
    echo inject-modules failed met exit-code %RC%
    exit /b %RC%
)

echo.
echo [inject-modules.bat] klaar. rootfs is bijgewerkt; flash 'm via UUU.
endlocal
