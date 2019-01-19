lib_serial_add_architecture("linux")

if("${LIB_SERIAL_ARCHITECTURE}" STREQUAL "linux")
    lib_serial_add_sourcefile_c(src/lib_serial_linux.c)
    lib_serial_add_header(include)
    lib_serial_add_public_definition(LIB_SERIAL_LINUX)
endif()

