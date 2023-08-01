PROJECT_NAME = ism330

MAKEFLAGS += --no-print-directory

compile:
	cmake --build ./build

upload:
	st-flash --reset write ./bin/$(PROJECT_NAME).bin 0x8000000