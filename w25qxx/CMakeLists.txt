idf_component_register(
    SRCS 
        "src/driver_w25qxx.c"
        "src/driver_w25qxx.h"
        "src/driver_w25qxx_interface.c"
        "src/driver_w25qxx_interface.h"
        "src/driver_w25qxx_advance.c"
        "src/driver_w25qxx_advance.h"
    INCLUDE_DIRS "src"
    REQUIRES driver spi
) 