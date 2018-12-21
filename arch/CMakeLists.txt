
######################################################################################
# Global Registration of the available architectures
######################################################################################
FUNCTION(lib_ser_add_architecture)
FOREACH(ARG ${ARGV})
  set_property(GLOBAL APPEND PROPERTY LIB_SER_ARCHITECTURES ${ARG})
ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_architecture)

######################################################################################
# Add architecture specific source files  
######################################################################################
FUNCTION(lib_ser_add_sourcefile_c)
  FOREACH(ARG ${ARGV})
    SET_PROPERTY(GLOBAL APPEND PROPERTY LIB_SER_ARCH_SOURCE_C ${CMAKE_CURRENT_SOURCE_DIR}/${ARG})
  ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_sourcefile_c)

######################################################################################
# Add architecture specific internal header files  
######################################################################################
FUNCTION(lib_ser_add_internal_header)
  FOREACH(ARG ${ARGV})
    SET_PROPERTY(GLOBAL APPEND PROPERTY LIB_SER_ARCH_INTERNAL_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/${ARG})
  ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_internal_header)

######################################################################################
# Add architecture specific header files  
######################################################################################
FUNCTION(lib_ser_add_header)
  FOREACH(ARG ${ARGV})
    SET_PROPERTY(GLOBAL APPEND PROPERTY LIB_SER_ARCH_PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/${ARG})
  ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_header)

######################################################################################
# Add architecture specific private definitions
######################################################################################
FUNCTION(lib_ser_add_public_definition)
  FOREACH(ARG ${ARGV})
    set_property(GLOBAL APPEND PROPERTY LIB_SER_ARCH_PUBLIC_DEFINITION ${ARG})
  ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_public_definition)

######################################################################################
# Add architecture specific dependencies
######################################################################################
FUNCTION(lib_ser_add_dependencies)
  FOREACH(ARG ${ARGV})
    set_property(GLOBAL APPEND PROPERTY LIB_SER_ARCH_DEPEND ${ARG})
  ENDFOREACH(ARG)
ENDFUNCTION(lib_ser_add_dependencies)


#add_subdirectory(linux)
add_subdirectory(stm32)
