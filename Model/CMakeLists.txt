################################################################################
# @file        CMakeLists.txt
# @brief       Top-level CMake build configuration for the PID Simulator.
#
# @details     Builds:
#                - pidsim_lib      : static library (all business logic)
#                - pidsim          : main CLI executable
#                - pidsim_tests    : GoogleTest unit test binary
#
#              Requirements:
#                - CMake >= 3.16
#                - C++17 compiler (GCC 9+, Clang 9+, MSVC 2019+)
#                - GoogleTest (fetched automatically via FetchContent)
#
#              Usage:
#                cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
#                cmake --build build
#                ctest --test-dir build --output-on-failure
#
#              Coverage (GCC/Clang):
#                cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug \
#                      -DENABLE_COVERAGE=ON
#                cmake --build build
#                ctest --test-dir build
#                # Then use gcovr or lcov to generate HTML report.
#
# @author      Matt Palmer
# @date        2026-05-01
# @version     0.1.0
################################################################################

cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(PIDSimulator
    VERSION     0.1.0
    DESCRIPTION "Industrial PID simulation controller"
    LANGUAGES   CXX
)

# ── C++ Standard ──────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD          17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ── Build type default ────────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ── Compiler warnings ─────────────────────────────────────────────────────────
add_compile_options(
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wextra>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wpedantic>
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# ── Optional coverage instrumentation ─────────────────────────────────────────
option(ENABLE_COVERAGE "Enable gcov coverage instrumentation" OFF)
if(ENABLE_COVERAGE)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        add_compile_options(--coverage -O0 -g)
        add_link_options(--coverage)
        message(STATUS "Coverage instrumentation enabled.")
    else()
        message(WARNING "Coverage is only supported with GCC or Clang.")
    endif()
endif()

# ── Source files for the business-logic library ───────────────────────────────
set(LIB_SOURCES
    Src/PID.cpp
    Src/Plant.cpp
    Src/Simulator.cpp
    Src/CLI.cpp
)

set(LIB_HEADERS
    Inc/Types.hpp
    Inc/PID.hpp
    Inc/Plant.hpp
    Inc/Simulator.hpp
    Inc/CLI.hpp
)

# ── pidsim_lib — static library ───────────────────────────────────────────────
add_library(pidsim_lib STATIC ${LIB_SOURCES} ${LIB_HEADERS})

target_include_directories(pidsim_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Inc>
        $<INSTALL_INTERFACE:Inc>
)

# ── pidsim — main executable ──────────────────────────────────────────────────
add_executable(pidsim
    Src/main.cpp
)

target_link_libraries(pidsim PRIVATE pidsim_lib)

# ── GoogleTest via FetchContent ───────────────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)

# Prevent GoogleTest from overriding our compiler/linker settings on Windows.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

# ── pidsim_tests — unit test binary ──────────────────────────────────────────
enable_testing()

set(TEST_SOURCES
    Test/test_PID.cpp
    Test/test_Plant.cpp
    Test/test_Simulator.cpp
    Test/test_CLI.cpp
)

add_executable(pidsim_tests ${TEST_SOURCES})

target_link_libraries(pidsim_tests
    PRIVATE
        pidsim_lib
        GTest::gtest_main
        GTest::gmock
)

target_include_directories(pidsim_tests
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
)

# Register tests with CTest.
include(GoogleTest)
gtest_discover_tests(pidsim_tests)