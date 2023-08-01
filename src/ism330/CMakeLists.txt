add_library(ISM330lib OBJECT 
    ism330.cpp
)

target_include_directories(ISM330lib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(ISM330lib PRIVATE STM32F446)