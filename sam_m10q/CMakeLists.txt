idf_component_register(
    SRCS 
        "src/driver_SAM_M10Q.c"
        "examples/sam_m10q_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
        "${CMAKE_CURRENT_SOURCE_DIR}/.."
    REQUIRES driver
) 