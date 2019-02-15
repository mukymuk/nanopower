@echo off
"%CROSSWORKS%\bin\crossbuild.exe" %1 -config "THUMB Debug" ..\cw\nanopower_meter_demo.hzp
