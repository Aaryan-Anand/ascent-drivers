idf_component_register(
    SRCS 
        "src/driver_H3LIS331DL.c"   
        "examples/lis331_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver i2c
) 