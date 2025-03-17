idf_component_register(
    SRCS 
        "src/driver_bno055.c"
    INCLUDE_DIRS "src"
    REQUIRES driver i2c
) 