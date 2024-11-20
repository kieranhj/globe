@echo off

if NOT EXIST build mkdir build

echo Assembling code...
bin\vasmarm_std_win32.exe -L build\compile.txt -m250 -Fvobj -opt-adr -o build\globe.o globe.asm

if %ERRORLEVEL% neq 0 (
	echo Failed to assemble code.
	exit /b 1
)

echo Linking code...
bin\vlink.exe -T vlink_script.txt -b rawbin1 -o build\globe.bin build\globe.o -Mbuild\linker.txt

if %ERRORLEVEL% neq 0 (
	echo Failed to link code.
	exit /b 1
)

echo Copying exe to Arculator hostfs folder...
set HOSTFS=..\arculator\hostfs
copy build\globe.bin "%HOSTFS%\globe,ff8"
