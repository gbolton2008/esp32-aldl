idf_component_register(SRCS "main.c" "decoder.c"
                       INCLUDE_DIRS ".")

if(EXISTS "${PROJECT_DIR}/version.txt")
    file(READ "${PROJECT_DIR}/version.txt" PROJECT_VER_CONTENT)
    string(STRIP "${PROJECT_VER_CONTENT}" PROJECT_VER_CONTENT)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE PROJECT_VER="${PROJECT_VER_CONTENT}")
endif()
