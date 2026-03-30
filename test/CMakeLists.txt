# Host-side unit tests (no cross-compilation, no hardware)
# Build: cmake -DBUILD_TESTS=ON .. && make && ctest

# Override cross-compilation settings for host build
set(CMAKE_SYSTEM_NAME "")
set(CMAKE_SYSTEM_PROCESSOR "")
set(CMAKE_C_COMPILER cc)
set(CMAKE_C_FLAGS "-Wall -Wextra -g -O0 -DHOST_TEST")

add_executable(test_pid
    test_pid.c
    ${CMAKE_SOURCE_DIR}/src/pid.c
)

target_include_directories(test_pid PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(test_pid m)

add_test(NAME pid_tests COMMAND test_pid)
