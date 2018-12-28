@echo off

if exist .\Output\ (
    rmdir /S /Q .\Output\
)
mkdir .\Output\

echo Compiling binary...
D:\x86_64-7.3.0-release-win32-seh-rt_v5-rev0\bin\g++.exe -std=c++11 -static-libstdc++ -static-libgcc -O3 -B ./ -DNGV_SYS_VERSION="""%date:~2,2%%date:~5,2%%date:~8,2%""" -DDX_GCC_COMPILE -DDX_NON_INLINE_ASM ./Drivers/NGH/*.c ./Drivers/NSASM/*.cpp ./Drivers/NSGDX/*.cpp *.cpp -LDrivers/DxLib -lDxLib -lDxUseCLib -lDxDrawFunc -ljpeg -lpng -lzlib -ltiff -ltheora_static -lvorbis_static -lvorbisfile_static -logg_static -lopusfile -lopus -lsilk_common -lcelt -lm -lgdi32 -o ./Output/ngh-emu
echo Done

echo Copying assets...
xcopy /Y /E /Q /I Assets Output\Assets

pause
