file(GLOB_RECURSE SOURCES "*.c" "*.cc" "*.h")

message("### ${CMAKE_PROJECT_NAME} ##")
foreach (SOURCE IN LISTS SOURCES)
    message("Add: ${SOURCE}")
endforeach ()

idf_component_register(SRCS ${SOURCES}
        INCLUDE_DIRS ".")

component_compile_options(
        -mtext-section-literals
        -Wall
        -Wextra
        -Wdouble-promotion
        -Wfloat-equal)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++23")
