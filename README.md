# RGBI2VGA
Simple, single file, 4 bit digital RGBI to analog VGA via an ESP32.

This ESP32 project, for the arduino development environment, converts the European Commodore 128 RGBI signal to a 640x400 @60Hz VGA signal.

It is in no way complete, but seems to work fine for running the default 80-column graphics/text-mode and GEOS 128.

The input signal sampling is performed via parallel I2S at 32MHz on core 1
and the VGA ouput is handled by running FabGL http://www.fabglib.org/ , https://github.com/fdivitto/FabGL on core 0.

