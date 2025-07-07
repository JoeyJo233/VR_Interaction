
::  Under Windows, starts two instances (nodes) of the default
::  solution in lab2 of TNM116, using the configuration that sets up
::  VRPN tracking,

call K:\TNM116\gramods-win\setenv.bat

start .\bin\main.exe --config config/desktop-2node-VRPNtracking.xml --param SyncNode.localPeerIdx=0

start .\bin\main.exe --config config/desktop-2node-VRPNtracking.xml --param SyncNode.localPeerIdx=1

pause
