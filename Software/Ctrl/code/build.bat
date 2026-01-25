@echo off

REM Debug Flags
REM set CompilerFlags=/EHsc /Zo /Z7 /Od /W4 /std:c++20
set CompilerFlags=/EHsc /Zo /Z7 /Od /W4 /WX /std:c++20
set CommonLinkerFlags= -incremental:no -opt:ref user32.lib 
REM set CommonLinkerFlags= -incremental:no -opt:ref user32.lib gdi32.lib winmm.lib

REM Release Flags
REM set CompilerFlags=/EHsc /Zo /Z7 /O2 /WX /W4 /std:c++20

REM Linkers
REM set LinkerFlags=/link /out:Ctrl.exe

IF NOT EXIST .\build mkdir .\build
pushd .\build
cl %CompilerFlags% ..\code\ctrl.cpp /link %CommonLinkerFlags%
popd
