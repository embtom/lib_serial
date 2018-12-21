project(lib_ser)

cmake_minimum_required(VERSION 2.6)
add_subdirectory(arch)

######################################################################################
# Build configurator
######################################################################################

#Selector to choose architecture to build lib_thread
set(LIB_SER_ARCHITECTURE ${LIB_SER_ARCHITECTURE} CACHE STRING "Architecture to build lib_ser on")
get_property(architectures GLOBAL PROPERTY LIB_SER_ARCHITECTURES)
list(SORT architectures)
set_property(CACHE LIB_SER_ARCHITECTURE PROPERTY STRINGS None ${architectures})

#######################################################################################
# Check LIB_THREAD_ARCHITECTURE has a valid value
#######################################################################################
if("${LIB_SER_ARCHITECTURE}" STREQUAL "None")
message(FATAL_ERROR "No valid architecture chosen! Please check LIB_SER_ARCHITECTURE.")
endif("${LIB_SER_ARCHITECTURE}" STREQUAL "None")

#######################################################################################
# Get Architecture specific build paramters
#######################################################################################
GET_PROPERTY(LIB_SER_SOURCE_C            GLOBAL PROPERTY LIB_SER_ARCH_SOURCE_C)
GET_PROPERTY(LIB_SER_PUBLIC_HEADER       GLOBAL PROPERTY LIB_SER_ARCH_PUBLIC_HEADER)
GET_PROPERTY(LIB_SER_PRIVATE_HEADER      GLOBAL PROPERTY LIB_SER_ARCH_INTERNAL_HEADER)
GET_PROPERTY(LIB_SER_DEPEND              GLOBAL PROPERTY LIB_SER_ARCH_DEPEND)
GET_PROPERTY(LIB_SER_PUBLIC_DEFINITION  GLOBAL PROPERTY LIB_SER_ARCH_PUBLIC_DEFINITION)

SET(LIB_SER_PUBLIC_HEADER ${LIB_SER_PUBLIC_HEADER} include)
SET(LIB_SER_PRIVATE_HEADER ${LIB_SER_PRIVATE_HEADER})
SET(LIB_SER_SOURCE_C   ${LIB_SER_SOURCE_C} )

if("${LIB_SER_ARCHITECTURE}" STREQUAL "linux")    
    find_package(iotty REQUIRED)
    find_package(osbase REQUIRED)
endif()

######################################################
#Build target settings
######################################################

add_library(${PROJECT_NAME} STATIC ${LIB_SER_SOURCE_C})
target_link_libraries(${PROJECT_NAME} ${LIB_SER_DEPEND})
target_include_directories(${PROJECT_NAME} PUBLIC ./include ${LIB_SER_PUBLIC_HEADER})
target_include_directories(${PROJECT_NAME} PRIVATE ${LIB_SER_PRIVATE_HEADER})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${LIB_SER_PUBLIC_DEFINITION})

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -E")
