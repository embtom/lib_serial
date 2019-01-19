project(lib_serial)

cmake_minimum_required(VERSION 2.6)
add_subdirectory(arch)

######################################################################################
# Build configurator
######################################################################################

#Selector to choose architecture to build lib_thread
set(LIB_SERIAL_ARCHITECTURE ${LIB_SERIAL_ARCHITECTURE} CACHE STRING "Architecture to build lib_ser on")
get_property(architectures GLOBAL PROPERTY LIB_SERIAL_ARCHITECTURES)
list(SORT architectures)
set_property(CACHE LIB_SERIAL_ARCHITECTURE PROPERTY STRINGS None ${architectures})

#######################################################################################
# Check LIB_THREAD_ARCHITECTURE has a valid value
#######################################################################################
if("${LIB_SERIAL_ARCHITECTURE}" STREQUAL "None")
    message(FATAL_ERROR "No valid architecture chosen! Please check LIB_SERIAL_ARCHITECTURE.")
endif("${LIB_SERIAL_ARCHITECTURE}" STREQUAL "None")

#######################################################################################
# Get Architecture specific build paramters
#######################################################################################
GET_PROPERTY(LIB_SERIAL_SOURCE_C            GLOBAL PROPERTY LIB_SERIAL_ARCH_SOURCE_C)
GET_PROPERTY(LIB_SERIAL_PUBLIC_HEADER       GLOBAL PROPERTY LIB_SERIAL_ARCH_PUBLIC_HEADER)
GET_PROPERTY(LIB_SERIAL_PRIVATE_HEADER      GLOBAL PROPERTY LIB_SERIAL_ARCH_INTERNAL_HEADER)
GET_PROPERTY(LIB_SERIAL_DEPEND              GLOBAL PROPERTY LIB_SERIAL_ARCH_DEPEND)
GET_PROPERTY(LIB_SERIAL_PUBLIC_DEFINITION  GLOBAL PROPERTY LIB_SERIAL_ARCH_PUBLIC_DEFINITION)

SET(LIB_SERIAL_PUBLIC_HEADER ${LIB_SERIAL_PUBLIC_HEADER} include)
SET(LIB_SERIAL_PRIVATE_HEADER ${LIB_SERIAL_PRIVATE_HEADER})
SET(LIB_SERIAL_SOURCE_C   ${LIB_SERIAL_SOURCE_C} )


######################################################
#Build target settings
######################################################
add_library(${PROJECT_NAME} STATIC ${LIB_SERIAL_SOURCE_C})
target_link_libraries(${PROJECT_NAME} ${LIB_SERIAL_DEPEND})
target_include_directories(${PROJECT_NAME} PUBLIC include ${LIB_SERIAL_PUBLIC_HEADER})
target_include_directories(${PROJECT_NAME} PRIVATE ${LIB_SERIAL_PRIVATE_HEADER})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${LIB_SERIAL_PUBLIC_DEFINITION})

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -E")
unset(LIB_SERIAL_ARCHITECTURE CACHE)