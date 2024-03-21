@echo off
setlocal

rem Get the full path of the current directory

rem Check if both source and destination paths are provided
if "%~1"=="" (
    echo Error: Source file path not provided.
    goto :eof
)

if "%~2"=="" (
    echo Error: Destination file path not provided.
    goto :eof
)

rem Set source and destination paths
set "sourceFilePath=%1"
set "destinationFilePath=%2"

set "currentDirectory=%CD%"

echo Full Path: %currentDirectory%

rem Extract the current directory name only
for %%I in ("%currentDirectory%") do set "currentDirectoryName=%%~nI"

rem Transfer folder from Raspberry Pi to local machine
scp -r pi@raspberrypi5:/home/pi/Desktop/meca500/%currentDirectoryName%/%sourceFilePath% %destinationFilePath%

echo Folder %sourceFilePath% transferred from Raspberry Pi to local machine.
