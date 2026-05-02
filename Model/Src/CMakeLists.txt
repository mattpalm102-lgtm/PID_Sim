################################################################################
# @file        Src/CMakeLists.txt
# @brief       Build configuration for core PID simulation sources.
#
# @details     Builds:
#                - pidsim_lib : static library (PID, Plant, Simulator)
#                - pidsim     : CLI executable (main + CLI interface)
#
#              This module is responsible for compiling all core simulation
#              logic and linking it into the application layer.
#
# @author      Matt Palmer
# @date        2026-05-01
# @version     0.1.0
################################################################################

# ── Source files for core library ─────────────────────────────────────────────
set(LIB_SOURCES
    PID.cpp
    Plant.cpp
    Simulator.cpp
)

# ── pidsim_lib — static library ──────────────────────────────────────────────
add_library(pidsim_lib STATIC ${LIB_SOURCES})

target_include_directories(pidsim_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/Inc>
        $<INSTALL_INTERFACE:Inc>
)

# Compiler warnings (target-specific)
target_compile_options(pidsim_lib PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# ── pidsim — CLI executable ──────────────────────────────────────────────────
add_executable(pidsim
    main.cpp
    CLI.cpp
)

target_link_libraries(pidsim
    PRIVATE
        pidsim_lib
)

# Apply same warnings to executable
target_compile_options(pidsim PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)