idf_component_register(
    SRCS 
        "src/driver_bno055.c"
        "examples/bno_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver i2c
) 