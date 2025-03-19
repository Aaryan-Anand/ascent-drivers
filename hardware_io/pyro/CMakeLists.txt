idf_component_register(
    SRCS 
        "src/pyro.c"
        "examples/pyro_test.c"
    INCLUDE_DIRS 
        "src"
        "examples"
    REQUIRES driver
) 