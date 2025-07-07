
:: Under Windows, starts the default solution in interaction exercise
:: of TNM116, using the configuration that sets up VRPN tracking

call C:\Apps\gramods.bin-TNM116-HT24-1\setenv.bat

rem start .\build_win\main.exe --config urn:gramods:config/se.liu.vortex.workbench-secondary-3DTV.xml
rem start .\build_win\main.exe --config urn:gramods:config/se.liu.vortex.workbench-master.xml

bin\main.exe --config urn:gramods:config/se.liu.workbench.3DTV-Sony50.xml

pause
