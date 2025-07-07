
::  Under Windows, starts two instances (nodes) of the default
::  solution in lab2 of TNM116, using the configuration that sets up
::  VRPN tracking,

call K:\TNM116\gramods-win\setenv.bat
cd /d %~dp0

.\bin\main.exe --config config/desktop-single-VRPNtracking.xml

pause
