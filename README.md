## ISM330DHCX
**Example usage of ISM330DHCX sensor and some of its internal features.**

*It was my experiment with some concepts, never intended to publish. Later I decided to finish it. I mostly code bare-metal stuff in C but here for reasons I don't remember is C++.*

### Some features:
- **Fully developed I2C with DMA** (for reading), error detection, write verification, bus recovery, retries.
- **Data noise reduction** with offset calibration, noise threshold, kalman filter.
- **Interrupt-driven measurment** of gyroscope data for proper integration with internal timer to compute d_t.
- **Non-blocking state machine** for Reading gyroscope, accelerometer and sensor fusion.
- Tap detection and pedometer.

### Possible improvements:
- I think my read->update->verify mechanism can be done differently. Manually setting clear mask manually is annoying.

*Sensor's documentation: https://www.st.com/resource/en/datasheet/ism330dhcx.pdf*

*Application note: https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-3d-accelerometer-and-3d-gyroscope-with-digital-output-for-industrial-applications-stmicroelectronics.pdf*

### Harware used:
- STM32F446RE
- Adafruit ISM330DHCX module

### Build, compile and upload:
1. In root `CMakeLists.txt` change the CMSIS directory or copy CMSIS to `CMSIS` folder. Furthermore, if you have a different board you must update the files, see the structure of `STM32F446RE` folder.
2. **CMake build:** on Linux run `build_cmake.sh`, or `cmake -S . -B build`.
3. **Compile:** run `make compile`. The binaries will be in `bin` folder.
4. **Upload:** run `make upload`. Remember to set the ST-Link directory if you are using windows. Remember to change the file name in makefile if you change project name in CMake.

*Unfortunately, Finite State Machine and Machine Learning Core is not included in this repository. Maybe I will do them in future.*
