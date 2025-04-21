## ISM330DHCX
**Example usage of ISM330DHCX sensor and some of its internal features.**

**It's an unfinished code, it's more an old experiment I decided to publish.**

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
