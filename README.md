# CSE-121-ESP-Climate-Monitor-Post-To-Web

This program is configured to run on an ESP32-C3 as specified by the Rust on ESP32 DevKit (https://docs.esp-rs.org/esp-rust-board/). The ESP must be connected via I2C to an SHT30 temperature and humidity sensor. Be sure to align the #define macros at the top of the main file with the appropriate GPIO pin numbers. Additionally, it may be necessary to alter the SHT30's I2C address, which also has a macro at the top of lab_7.c. Also change the HTTP server parameters #defined at the top of the source file. 

The ESP will continously wake the SHT30, read its fresh measurements, and use HTTP POST to submit them to the server. This is done in real time using the ESP's ability to run FreeRTOS. The SHT30's measurements will be checked for accuracy via CRC calculation.

Make sure your system has the required tools:

```bash
sudo apt-get install fish neovim g++ git wget \
   flex bison gperf python3 python3-venv cmake \
   ninja-build ccache libffi-dev libssl-dev \
   dfu-util libusb-1.0-0
```

You will also need the esp-idf library up and running in your CWD. 