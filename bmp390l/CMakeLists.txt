idf_component_register(
    SRCS 
        "src/driver_BMP390L.c"
    INCLUDE_DIRS "src"
    REQUIRES driver i2c
) 