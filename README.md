# twai-obd2-simulator

This is a simple OBD2 simulator that can be used to test OBD2 applications. It
uses [TWAI](https://docs.espressif.com/projects/esp-idf/en/v4.2/esp32/api-reference/peripherals/twai.html) interface,
therefore can be run on ESP32.

This work is based on [esp32-obd2-emulator](https://github.com/limiter121/esp32-obd2-emulator).

## Usage

Use [PlatformIO](https://platformio.org/) to build and upload the code to your ESP32.

Or, you might want to rename [main.cpp](src/main.cpp) to have an `.ino` extension and run it on Arduino IDE.
No extra libraries required. Just make sure you have the ESP32 board installed and the right board selected on the IDE configuration.

## Supported PID's

### Mode 1

- 0x00 - PIDs supported
- 0x0C - Engine RPM
- 0x0D - Vehicle speed

### Mode 9

- 0x00 - PIDs supported
- 0x02 - VIN

## Values

This simulator uses a fixed value. You can change it on the [main.cpp](src/main.cpp) file.

## Baudrate

The default baudrate is 25kbps. You can change it on the [main.cpp](src/main.cpp) file.