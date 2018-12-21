lib_ser_add_architecture("stm32")

if("${LIB_SER_ARCHITECTURE}" STREQUAL "stm32")
    lib_ser_add_dependencies(lib_convention lib_stm32hal lib_FREERTOS lib_log lib_isr lib_timer)
    lib_ser_add_sourcefile_c(src/lib_ser_STM32.c)
    lib_ser_add_header(include)
    lib_ser_add_public_definition(LIB_SER_STM32)
endif()

