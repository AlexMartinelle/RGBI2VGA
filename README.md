# RGBI2VGA
Simple, single file, 4 bit digital RGBI to analog VGA via an ESP32.

This ESP32 project, for the arduino development environment, converts the European Commodore 128 RGBI signal to a 640x200@70Hz VGA signal.

It is in no way complete, but seems to work fine for running the default 80-column graphics/text-mode and GEOS 128.
It can probably be adapted to handle NTSC C128 & CGA outputs as well.

The input signal sampling is performed via parallel I2S at 32MHz on core 1
and the VGA ouput is handled by running FabGL http://www.fabglib.org/ , https://github.com/fdivitto/FabGL on core 0.

Remember to switch the core for FabGLs (In fabglconf.h: Set the "#define FABGLIB_VIDEO_CPUINTENSIVE_TASKS_CORE 0" instead of WIFI_TASK_CORE_ID)

Don't trust the schematic, double check that all the pins on your ESP32 match what's in the code and that the connection makes sense.
It's version 0.1..
https://github.com/AlexMartinelle/RGBI2VGA/blob/main/RGBI2VGAESP32Schemagic.png
