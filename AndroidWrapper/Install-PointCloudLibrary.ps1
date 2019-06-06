<#
.SYNOPSIS
Retrieves and extracts the PointCloudLirary from the "Appveyor" page.
.EXAMPLE
Get-PointCloudLibrary
#>

# 3.0?
# Import-Module "PS-Zip.psm1" -Force

Write-Host $args[0]
$architecture = $args[0]

# Full versioned pack file name to download
# $architecture = "x86"
# $architecture = "x86_64"
# $architecture = "armeabi-v7a"
# $architecture = "arm64-v8a"

$zipFilePath = "pcl-superbuild-$architecture.zip"

# base URL to download the pack file from
if ($architecture -eq "x86")
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/lrsfpvvy7d7576y0/artifacts/$zipFilePath"
}
elseif ($architecture -eq "x86_64")
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/91yjggxwteidcopw/artifacts/$zipFilePath"
}
elseif ($architecture -eq "arm64-v8a")
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/ha0lfikov8q4my0o/artifacts/$zipFilePath"
}
elseif ($architecture -eq "armeabi-v7a")
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/64oer708j0dr3c1l/artifacts/$zipFilePath"
}
elseif ($architecture -eq "armeabi")
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/ /artifacts/$zipFilePath"
}
else
{
    $SourceURLBase = "https://ci.appveyor.com/api/buildjobs/64oer708j0dr3c1l/artifacts/pcl-superbuild-armeabi-v7a.zip"
    $architecture = "armeabi-v7a"
}

# download the pack and extract the files into the curent directory 
# How to get the current directory of the cmdlet being executed
# http://stackoverflow.com/questions/8343767/how-to-get-the-current-directory-of-the-cmdlet-being-executed
$dstPath = (Get-Item -Path ".\" -Verbose).FullName
$dstFile = $zipFilePath

# Version Check
# PowerShell Version 2.0
# 1.0 Blank
# $PSVersionTable

# check if path already exists
$ArchitectPathExists = Test-Path (Join-Path $dstPath $dstFile) -ErrorAction SilentlyContinue
If($ArchitectPathExists -eq $False)
{
    # Download pcl_superbuild
    # Write $SourceURLBase
    # PowerShell Version 3.0
    # Invoke-WebRequest -UseBasicParsing -Uri $packSourceURLBase | Expand-Stream -Destination $dstPath
    # 2.0
    $cli = New-Object System.Net.WebClient
    $cli.DownloadFile($SourceURLBase, (Join-Path $dstPath $dstFile))
}

# Extract zip File
# 3.0
# New-ZipExtract -source $zipFilePath -destination $dstPath -force -verbose
# 2.0
$shell = New-Object -ComObject shell.application
$zip = $shell.NameSpace((Join-Path $dstPath $dstFile))
$dstLibPath = $dstPath + "/aar/pclmobile/libs/$architecture"
Write-Host $dstLibPath
New-Item $dstLibPath -type directory -Force
$dest = $shell.NameSpace((Split-Path (Join-Path $dstLibPath $dstFile) -Parent))
$dest.CopyHere($zip.Items()) 

# # Copy binary
# # Copy-Item $dstPath/bin/* $dstPath
# $dstLibPath = $dstPath + "/aar/pclmobile/libs/$architecture"
# Write-Host $dstLibPath
# # If there is an existing folder, the existing folder will not change anything in particular, but no error will be issued.
# Move-Item $dstPath/boost-android $dstLibPath
# Move-Item $dstPath/eigen $dstLibPath
# Move-Item $dstPath/flann-android $dstLibPath
# Move-Item $dstPath/pcl-android $dstLibPath
# Move-Item $dstPath/qhull-android $dstLibPath

