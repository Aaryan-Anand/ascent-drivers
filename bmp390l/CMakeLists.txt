idf_component_register(
    SRCS 
        "src/driver_BMP390L.c"
        "examples/bmp_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
    REQUIRES driver i2c
) 