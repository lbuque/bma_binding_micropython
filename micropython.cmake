# Create an INTERFACE library for our C module.
add_library(usermod_bma INTERFACE)

include(${CMAKE_CURRENT_LIST_DIR}/depend.cmake)

file(GLOB MOD_SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.c)
file(GLOB BMA423_Sensor_API_SRC ${CMAKE_CURRENT_LIST_DIR}/lib/BMA423-Sensor-API/*.c)

# Add our source files to the lib
target_sources(usermod_bma INTERFACE
    ${MOD_SRC}
    ${BMA423_Sensor_API_SRC}
)

# Add the current directory as an include directory.
target_include_directories(usermod_bma INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/lib/BMA423-Sensor-API
)

# Link our INTERFACE library to the usermod target.
target_link_libraries(usermod INTERFACE usermod_bma)
