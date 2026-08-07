@echo off
REM test_server_callback.bat
REM Usage: test_server_callback.bat <server_ip> [port]
REM Example: test_server_callback.bat 203.0.113.42 8081

SETLOCAL ENABLEEXTENSIONS ENABLEDELAYEDEXPANSION

REM Default values
SET SERVER_IP=%1
IF "%SERVER_IP%"=="" SET SERVER_IP=137.184.35.110
SET PORT=%2
IF "%PORT%"=="" SET PORT=8081

REM Sample RockBLOCK POST data (hex payload)
REM Previous payload (old, from Dry land test): 120012070d6ff0a53f107e
REM Sample payload:
SET DATA_HEX=0a140d3d0a4542150080f6c21d00002040250000344312070d0000a040105a12080d0000f04010c8011a0a0d66666641159a9999bf222608b96015000044421d0000f6c225000040402d0000344235000000003d00000040450000c0402d0000a441359a9901413d00000c424580e6c5474a180a0a0d9a994342153333f6c20a0a0d66664442159a99f6c2

REM Sample values for other RockBLOCK fields
SET IMEI=300434065264590
SET MOMSN=1234
REM Current UTC timestamp, generated at run time
for /f "delims=" %%i in ('powershell -NoProfile -Command "([DateTime]::UtcNow).ToString('yyyy-MM-ddTHH:mm:ssZ')"') do set "TRANSMIT_TIME=%%i"
SET IRIDIUM_LATITUDE=37.7749
SET IRIDIUM_LONGITUDE=-122.4194
SET IRIDIUM_CEP=9
SET SERIAL=5678

SET POST_BODY=imei=%IMEI%^&serial=%SERIAL%^&momsn=%MOMSN%^&transmit_time=%TRANSMIT_TIME%^&iridium_latitude=%IRIDIUM_LATITUDE%^&iridium_longitude=%IRIDIUM_LONGITUDE%^&iridium_cep=%IRIDIUM_CEP%^&data=%DATA_HEX%

curl -v -X POST ^
  -H "Content-Type: application/x-www-form-urlencoded" ^
  -d "%POST_BODY%" ^
  "http://%SERVER_IP%:%PORT%/sensors"
