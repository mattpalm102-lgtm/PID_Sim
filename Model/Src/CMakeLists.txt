# Core library (reusable simulation engine)
add_library(pid_model
    PID.cpp
    Plant.cpp
    Simulator.cpp
)

target_include_directories(pid_model
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/../Inc
)

# Compiler warnings
if (MSVC)
    target_compile_options(pid_model PRIVATE /W4 /permissive-)
else()
    target_compile_options(pid_model PRIVATE -Wall -Wextra -Wpedantic)
endif()


# CLI executable (application layer)
add_executable(pid_cli
    main.cpp
    CLI.cpp
)

target_link_libraries(pid_cli
    PRIVATE pid_model
)