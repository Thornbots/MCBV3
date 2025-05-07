cd "$env:USERPROFILE"

# Install stlink driver
.\en.stsw-link009_1\stlink_winusb_install.bat

if(!([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] 'Administrator')) {
 Start-Process -FilePath PowerShell.exe -Verb Runas -ArgumentList "-File `"$($MyInvocation.MyCommand.Path)`"  `"$($MyInvocation.MyCommand.UnboundArguments)`""
 Exit
}

# Use chocolatey for any packages it supports. Makes job easier
$chocoexists = Get-Command -Name choco.exe -ErrorAction SilentlyContinue
if(-not($chocoexists)){
    Write-Output "Chocolatey is not installed, installing now"
    # From chocolatey.org/install
    Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))
} else {
    Write-Output "Chocolatey is already installed"
}

# Make `refreshenv` available right away, by defining the $env:ChocolateyInstall
# variable and importing the Chocolatey profile module.
# Note: Using `. $PROFILE` instead *may* work, but isn't guaranteed to.
$env:ChocolateyInstall = Convert-Path "$((Get-Command choco).Path)\..\.."   
Import-Module "$env:ChocolateyInstall\helpers\chocolateyProfile.psm1"


refreshenv # to update enviromen vars

# mingw contains gcc and python3.9
choco install -y python38 git vscode gcc-arm-embedded openocd mingw

git clone https://github.com/Thornbots/MCBV3.git
cd MCBV3

pip3 install pipenv
cd MCB-project
pipenv install

refreshenv # to update enviromen vars

# installing vscode extensions
$json_file = Get-Content $PWD\.vscode\extensions.json | Out-String | ConvertFrom-Json
foreach($elem in $json_file.PsObject.Properties.Value){
    Write-Host $elem # print
    code --install-extension $elem
}

Start-Process -NoNewWindow code .
exit 0
