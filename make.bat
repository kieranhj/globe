@echo off

if NOT EXIST build mkdir build

echo Assembling code...
bin\vasmarm_std_win32.exe -L compile.txt -m250 -Fbin -opt-adr -o build\globe.bin globe.asm

if %ERRORLEVEL% neq 0 (
	echo Failed to assemble code.
	exit /b 1
)

echo Copying exe to Arculator hostfs folder...
set HOSTFS=..\arculator\hostfs
copy build\globe.bin "%HOSTFS%\globe,ff8"
