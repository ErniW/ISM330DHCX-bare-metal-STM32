add_library(systicklib OBJECT 
    systick.cpp
)

target_include_directories(systicklib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(systicklib PRIVATE STM32F446)