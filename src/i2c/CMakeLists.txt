add_library(I2Clib OBJECT 
    i2c.cpp
)

target_include_directories(I2Clib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(I2Clib PRIVATE STM32F446)