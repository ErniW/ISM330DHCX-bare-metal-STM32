add_library(ISM330lib OBJECT 
    ism330.cpp
    quaternions.cpp
    timer.cpp
    kalman.cpp
)

target_include_directories(ISM330lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/Fusion
)

target_link_libraries(ISM330lib PRIVATE 
    STM32F446 
    I2Clib
    systicklib
    FUSIONlib
)