## ISM330DHCX
**Example usage of ISM330DHCX sensor and some of its internal features.**

*Sensor's documentation: https://www.st.com/resource/en/datasheet/ism330dhcx.pdf*

*Application note: https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-3d-accelerometer-and-3d-gyroscope-with-digital-output-for-industrial-applications-stmicroelectronics.pdf*

### Harware used:
- STM32F446RE
- Adafruit ISM330DHCX module

### Compile and upload:
- **GNU Make and arm-none-eabi-gcc toolchain is required.** Type `make compile` to compile.
- **To upload to your board use ST-Link.** In makefile change it's directory. Then type `make upload` in terminal. https://github.com/stlink-org/stlink

*Or you can copy and paste code into STM32CubeIDE.*

### Features covered:
- Accelerometer and gyroscope.
- IMU orientation.
- 6D tilt orientation.
- Free falling detection.
- Double tap detection.
- Pedometer.

*Unfortunately, Finite State Machine and Machine Learning Core is not included in this repository. Maybe I will do them in future.*
