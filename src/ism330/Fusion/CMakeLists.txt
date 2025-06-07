add_library(FUSIONlib OBJECT 
    FusionAhrs.c
    FusionCompass.c
    FusionOffset.c
)

target_include_directories(FUSIONlib PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)