# Generate our data logging module to handle all the message types
FIND_PACKAGE(PythonInterp 3 REQUIRED)

SET(source_file "${CMAKE_CURRENT_BINARY_DIR}/src/DataPlayback.cpp")
SET(config_file "${CMAKE_CURRENT_BINARY_DIR}/data/config/DataPlayback.yaml")

# Copy across the files
ADD_CUSTOM_COMMAND(
    USES_TERMINAL
    OUTPUT "${source_file}"
           "${config_file}"
    COMMAND ${PYTHON_EXECUTABLE}
    ARGS "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
         "${PROJECT_BINARY_DIR}/shared"
         "${source_file}"
         "${config_file}"
    DEPENDS ${NUCLEAR_MESSAGE_LIBRARIES}
            "${CMAKE_CURRENT_SOURCE_DIR}/src/generate_dataplayback.py"
    COMMENT "Generating DataPlayback system for current messages"
)

# Build our NUClear module
NUCLEAR_MODULE(SOURCES "${source_file}"
               DATA_FILES "${config_file}:config")
