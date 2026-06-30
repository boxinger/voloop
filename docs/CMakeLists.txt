find_package(Doxygen REQUIRED)

set(VOLOOP_DOXYGEN_OUTPUT_DIR "${PROJECT_SOURCE_DIR}/build/docs")

add_custom_target(voloop_docs
    COMMAND ${CMAKE_COMMAND} -E make_directory "${VOLOOP_DOXYGEN_OUTPUT_DIR}"
    COMMAND ${DOXYGEN_EXECUTABLE} "${PROJECT_SOURCE_DIR}/Doxyfile"
    WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
    COMMENT "Generating Doxygen documentation"
    VERBATIM
)
