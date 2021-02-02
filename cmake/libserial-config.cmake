get_filename_component(libserial_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
include(CMakeFindDependencyMacro)

list(APPEND CMAKE_MODULE_PATH "${libserial_CMAKE_DIR}/cmake")

if(NOT TARGET EMBTOM::libserial)
    include("${libserial_CMAKE_DIR}/libserial-targets.cmake")
endif()