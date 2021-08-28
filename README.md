# Nano-IoT-Environment-Logger

Using the Arduino Nano IoT 33 and either a BME280, SHTC3, or ADT7410 sensor send environmental
measurements to an MQTT broker.

## Features

### Multiple Sensors

The program supports one of several sensors.
- BME280 - temperature, humidity, and pressure
- SHTC3 - temperatue, humididy
- ADT7410 - temperature

### BLE Configuration
During the first minute after reset connect to the board with BLE to change the default configuration.
The configuration will be saved in flash memory.

Note, you must always set the WiFi password when updating the configuration.

### Battery Monitor

If enabled the charge level of the battery will be included in the MQTT message.

### LUA Script for BlueSee

Use BlueSee with the included LUA script for a simple UI to configure the board via BLE.