idf_component_register(
    SRCS 
        "src/driver_H3LIS331DL.c"
    INCLUDE_DIRS "src"
    REQUIRES driver i2c
) 