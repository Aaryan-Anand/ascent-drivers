idf_component_register(
    SRCS 
        "src/driver_w25qxx_basic.c"
        "src/driver_w25qxx_interface.c"
    INCLUDE_DIRS "src"
    REQUIRES driver spi
) 