
set(url "https://github.com/BoschSensortec/BMA423-Sensor-API.git")
set(commit "e65f82683cc2e0d2d4bd8dcfa14089c54bf8787d")

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib)
    execute_process(COMMAND mkdir -p ${CMAKE_CURRENT_LIST_DIR}/lib)
endif()

if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/lib/BMA423-Sensor-API)
    execute_process(COMMAND
                        git clone ${url}
                    WORKING_DIRECTORY
                        ${CMAKE_CURRENT_LIST_DIR}/lib)
endif()
