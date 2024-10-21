file(GLOB_RECURSE SOURCES_C *.c)
file(GLOB_RECURSE SOURCES_CPP *.cpp)

idf_component_register(
    SRCS "example_qspi_with_ram.c"
    INCLUDE_DIRS ".")
