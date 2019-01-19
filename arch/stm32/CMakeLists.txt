lib_serial_add_architecture("stm32")

if("${LIB_SERIAL_ARCHITECTURE}" STREQUAL "stm32")
    lib_serial_add_dependencies(lib_convention lib_stm32hal lib_FREERTOS lib_log lib_isr lib_timer)
    lib_serial_add_sourcefile_c(src/lib_serial_stm32.c 
                                src/serial_access_stm32.c)
    lib_serial_add_header(include)
    lib_serial_add_internal_header(internal_include)
    lib_serial_add_public_definition(LIB_SERIAL_STM32)
endif()

