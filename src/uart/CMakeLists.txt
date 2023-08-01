add_library(UARTlib OBJECT 
    uart.cpp
)

target_include_directories(UARTlib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(UARTlib PRIVATE STM32F446)