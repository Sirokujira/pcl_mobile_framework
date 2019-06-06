# install AndroidNDK under Windows
# Authors: 
# License: CC0 1.0 Universal: http://creativecommons.org/publicdomain/zero/1.0/

# https://dl.google.com/android/repository/android-ndk-r10e-windows-x86.zip
$BASE_ANDROIDNDK_URL = "https://dl.google.com/android/repository"


function Download ($filename, $url) 
{
    $webclient = New-Object System.Net.WebClient

    $basedir = $pwd.Path + "\"
    $filepath = $basedir + $filename
    if (Test-Path $filename) 
    {
        Write-Host "Reusing" $filepath
        return $filepath
    }

    # Download and retry up to 3 times in case of network transient errors.
    Write-Host "Downloading" $filename "from" $url
    $retry_attempts = 2
    for ($i = 0; $i -lt $retry_attempts; $i++) {
        try {
            $webclient.DownloadFile($url, $filepath)
            break
        }
        Catch [Exception]{
            Start-Sleep 1
        }
    }
    
    if (Test-Path $filepath) 
    {
        Write-Host "File saved at" $filepath
        Write-Host $(Get-ChildItem $filepath).Length
    }
    else 
    {
        # Retry once to get the error message if any at the last try
        $webclient.DownloadFile($url, $filepath)
    }
    
    return $filepath
}

function InstallAndroidNDK_ZIP ($zippath, $ndk_home, $install_log)
{
    New-ZipExtract -source $zippath -destination $ndk_home -force -verbose
}

function InstallAndroidNDK_EXE ($exepath, $ndk_home, $install_log)
{
    # old
    # $install_args = "/quiet InstallAllUsers=1 TargetDir=$ndk_home"
    # RunCommand $exepath $install_args
    
    # http://www.ibm.com/support/knowledgecenter/SS2RWS_2.1.0/com.ibm.zsecure.doc_2.1/visual_client/responseexamples.html?lang=ja
    $install_args = "/S /v/qn /v/norestart"
    # RunCommand schtasks /create /tn ndkinstall /RL HIGHEST /tr $exepath /S /v/norestart /v/qn /sc once /st 23:59
    # RunCommand schtasks /run /tn ndkinstall
    # RunCommand schtasks /delete /tn ndkinstall /f
    # RunCommand sleep 600
    RunCommand "schtasks" "/create /tn ndkinstall /RL HIGHEST /tr `"$exepath $install_args`" /sc once /st 23:59"
    RunCommand "sleep" "10"
    RunCommand "schtasks" "/run /tn ndkinstall"
    RunCommand "sleep" "300"
    RunCommand "schtasks" "/delete /tn ndkinstall /f"
}

function InstallAndroidNDK_MSI ($msipath, $ndk_home, $install_log)
{
    $install_args = "/qn /log $install_log /i $msipath TARGETDIR=$ndk_home"
    $uninstall_args = "/qn /x $msipath"
    RunCommand "msiexec.exe" $install_args
    if (-not(Test-Path $ndk_home)) 
    {
        Write-Host "AndroidNDK seems to be installed else-where, reinstalling."
        RunCommand "msiexec.exe" $uninstall_args
        RunCommand "msiexec.exe" $install_args
    }
}

function RunCommand ($command, $command_args) 
{
    Write-Host $command $command_args
    Start-Process -FilePath $command -ArgumentList $command_args -Wait -Passthru
}

function DownloadAndroidNDK ($version, $platform_suffix) 
{
    $filename = "android-ndk-" + $version + "-windows-" + "$platform_suffix" + ".zip"
    $url = "$BASE_ANDROIDNDK_URL" + "/" + "android-ndk-" + $version + "-windows-" + "$platform_suffix" + ".zip"

    # (plan modified function)
    $filepath = Download $filename $url
    return $filepath
}

function InstallAndroidNDK ($ndk_version, $architecture, $ndk_home) 
{
    Write-Host "InstallAndroidNDK"
    
    if ($architecture -eq "32")
    {
        $platform_suffix = "x86"
    }
    else
    {
        $platform_suffix = "x86_64"
    }
    
    # https://dl.google.com/android/repository/android-ndk-r10e-windows-x86.zip
    $installer_path = DownloadAndroidNDK $ndk_version $platform_suffix
    $installer_ext = [System.IO.Path]::GetExtension($installer_path)
    
    Write-Host "Installing $installer_path to $ndk_home"
    $install_log = $ndk_home + "install.log"
    if ($installer_ext -eq '.msi')
    {
        InstallAndroidNDK_MSI $installer_path $ndk_home $install_log
    }
    else
    {
        # InstallAndroidNDK_EXE $installer_path $ndk_home $install_log
        InstallAndroidNDK_ZIP $installer_path $ndk_home $install_log
    }
    
    if (Test-Path $ndk_home) 
    {
        Write-Host "ANDROID_NDK $ndk_version ($architecture) installation complete"
    }
    else 
    {
        Write-Host "Failed to install ANDROID_NDK in $ndk_home"
        # Get-Content -Path $install_log
        # Exit 1
    }
}

function main () 
{
    Write-Host "main - enter"
    
    # Android NDK
    # InstallAndroidNDK $env:NDK_VERSION $env:OS_ARCH $env:ANDROID_NDK
    InstallAndroidNDK $env:NDK_VERSION $env:OS_ARCH $env:ROOT
    
    Write-Host "main - exit"
}
