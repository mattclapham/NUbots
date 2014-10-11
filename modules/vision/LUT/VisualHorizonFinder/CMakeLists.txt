# Find the QUEX Library
FIND_PACKAGE(QUEX REQUIRED)

# Our lexer file
SET(lexer "src/VisualHorizonLexer.qx")

SET(lexer_class "VisualHorizonLexer")

# Get the filename without the extension
GET_FILENAME_COMPONENT(file_we ${lexer} NAME_WE)

# Get the absolute file location
GET_FILENAME_COMPONENT(abs_file ${lexer} ABSOLUTE)

# Work out our output path
FILE(RELATIVE_PATH outputpath ${CMAKE_CURRENT_SOURCE_DIR} ${abs_file})
GET_FILENAME_COMPONENT(outputpath ${outputpath} PATH)
SET(outputpath "${CMAKE_CURRENT_BINARY_DIR}/${outputpath}")

# Make our output directory
FILE(MAKE_DIRECTORY ${outputpath})

# Add the command to generate the lexer
# Make the quex
ADD_CUSTOM_COMMAND(
    OUTPUT "${outputpath}/${lexer_class}-token_ids.hpp"
           "${outputpath}/${lexer_class}.cpp"
           "${outputpath}/${lexer_class}.hpp"
           "${outputpath}/${lexer_class}-token.hpp"
           "${outputpath}/${lexer_class}-configuration.hpp"
    COMMAND "/usr/local/bin/quex"
    ARGS -i ${abs_file}
         --file-extension-scheme pp
         --output-directory ${outputpath}
         --analyzer-class ${lexer_class}
         --language C++
         --token-policy single
         --no-string-accumulator
         --single-mode-analyzer
         --no-mode-transition-check
         # --path-compression
         # --template-compression

    DEPENDS ${abs_file}
    COMMENT "Generating Quex lexer for vision"
    VERBATIM)

# Build our NUClear module
NUCLEAR_MODULE(SOURCES
    ${outputpath}/${lexer_class}-token_ids.hpp
    ${outputpath}/${lexer_class}.cpp
    ${outputpath}/${lexer_class}.hpp
    ${outputpath}/${lexer_class}-token.hpp
    ${outputpath}/${lexer_class}-configuration.hpp)