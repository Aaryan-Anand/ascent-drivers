idf_component_register(
    SRCS 
        "src/driver_H3LIS331DL.c"   
        "examples/lis331_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
    REQUIRES driver i2c
) 