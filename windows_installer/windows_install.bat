:: go to cwd of the batch file bc runnin batch file in admin mode goes somewhere else
cd %~dp0

echo --Install stlink driver--
call .\en.stsw-link009_1\stlink_winusb_install_quiet.bat

echo --install choco--
@"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "[System.Net.ServicePointManager]::SecurityProtocol = 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))" && SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"

call refreshenv

echo  --Install all deps--
choco install -y python38 git vscode gcc-arm-embedded openocd mingw jq

call refreshenv

echo  --clone repo--
git clone https://github.com/Thornbots/MCBV3.git
cd MCBV3

echo  --setup pipenv--
pip3 install pipenv
cd MCB-project
pipenv install

setlocal
echo --install vscode extentions--
cd ../
:: Path to the extensions.json file
set "extensionsFile=./.vscode/extensions.json"

:: Read the extensions from the JSON file
for /f "delims=" %%i in ('jq.exe -r ".recommendations[]" "%extensionsFile%"') do (
    call code --install-extension %%i
)

echo vscode Extensions installation completed.
endlocal

cd ../
code . | exit
