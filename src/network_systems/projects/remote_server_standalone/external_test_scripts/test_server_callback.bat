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
SET DATA_HEX=120012070d6ff0a53f107e

REM Sample values for other RockBLOCK fields
SET IMEI=300434065264590
SET MOMSN=1234
SET TRANSMIT_TIME=2026-03-28T12:00:00Z
SET IRIDIUM_LATITUDE=37.7749
SET IRIDIUM_LONGITUDE=-122.4194
SET IRIDIUM_CEP=9
SET SERIAL=5678

SET POST_BODY=imei=%IMEI%^&serial=%SERIAL%^&momsn=%MOMSN%^&transmit_time=%TRANSMIT_TIME%^&iridium_latitude=%IRIDIUM_LATITUDE%^&iridium_longitude=%IRIDIUM_LONGITUDE%^&iridium_cep=%IRIDIUM_CEP%^&data=%DATA_HEX%

curl -v -X POST ^
  -H "Content-Type: application/x-www-form-urlencoded" ^
  -d "%POST_BODY%" ^
  "http://%SERVER_IP%:%PORT%/sensors"
