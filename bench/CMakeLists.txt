# Reference implementation for parity tests and benchmarks. Never linked into the charqui library itself.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
FetchContent_Declare(ruckig
    GIT_REPOSITORY https://github.com/pantor/ruckig.git
    GIT_TAG v0.19.4
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(ruckig)

add_executable(parity parity.cpp)
target_link_libraries(parity PRIVATE charqui::charqui ruckig::ruckig)
target_compile_options(parity PRIVATE -Wno-missing-field-initializers)
add_test(NAME parity COMMAND parity 20000)

add_executable(bench_vs_ruckig bench_vs_ruckig.cpp)
target_link_libraries(bench_vs_ruckig PRIVATE charqui::charqui ruckig::ruckig)
target_compile_options(bench_vs_ruckig PRIVATE -Wno-missing-field-initializers)
