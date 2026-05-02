################################################################################
# @file        Test/CMakeLists.txt
# @brief       Unit test build configuration using GoogleTest.
#
# @details     Builds:
#                - pidsim_tests : unit test executable
#
#              Links against:
#                - pidsim_lib
#                - GoogleTest (gtest_main, gmock)
#
#              Tests are automatically discovered and registered with CTest.
#
# @author      Matt Palmer
# @date        2026-05-01
# @version     0.1.0
################################################################################

# ── Test sources ──────────────────────────────────────────────────────────────
set(TEST_SOURCES
    test_PID.cpp
    test_Plant.cpp
    test_Simulator.cpp
    test_CLI.cpp
)

# ── pidsim_tests — unit test executable ──────────────────────────────────────
add_executable(pidsim_tests ${TEST_SOURCES})

target_link_libraries(pidsim_tests
    PRIVATE
        pidsim_lib
        GTest::gtest_main
        GTest::gmock
)

target_include_directories(pidsim_tests
    PRIVATE
        ${CMAKE_SOURCE_DIR}/Inc
)

# Compiler warnings (same standard as main targets)
target_compile_options(pidsim_tests PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-Wall -Wextra -Wpedantic -Wshadow>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

# ── Register tests with CTest ─────────────────────────────────────────────────
include(GoogleTest)
gtest_discover_tests(pidsim_tests)