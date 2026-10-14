cmake_minimum_required(VERSION 3.24)
project(charqui VERSION 0.1.0 LANGUAGES CXX DESCRIPTION "Jerk-limited online trajectory generation with intermediate waypoints")

# ROS 2: ament_cmake is optional. Looked up first so that the Python bindings default to ON under colcon and the
# package can be exported at the bottom; plain CMake builds are unaffected.
find_package(ament_cmake QUIET)

option(CHARQUI_BUILD_TESTS "Build the C++ tests" ${PROJECT_IS_TOP_LEVEL})
option(CHARQUI_BUILD_EXAMPLES "Build the examples" ${PROJECT_IS_TOP_LEVEL})
option(CHARQUI_BUILD_BENCHMARK "Build the benchmark against Ruckig (fetches Ruckig)" OFF)
option(CHARQUI_BUILD_PYTHON "Build the nanobind Python module (default ON under ROS 2 / colcon)" ${ament_cmake_FOUND})
option(CHARQUI_INSTALL "Generate install rules for the C++ library" ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

add_library(charqui
    src/brake.cpp
    src/position_third_step1.cpp
    src/position_third_step2.cpp
    src/position_second_step1.cpp
    src/position_second_step2.cpp
    src/position_first_step1.cpp
    src/position_first_step2.cpp
    src/velocity_third_step1.cpp
    src/velocity_third_step2.cpp
    src/velocity_second_step1.cpp
    src/velocity_second_step2.cpp
    src/trajectory.cpp
    src/target_solver.cpp
    src/waypoint_solver.cpp
    src/planner.cpp
)
add_library(charqui::charqui ALIAS charqui)
target_include_directories(charqui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(charqui PUBLIC cxx_std_23)
if(MSVC)
    target_compile_options(charqui PRIVATE /W4)
else()
    # -fno-math-errno lets sqrt() inline to a single instruction and sin/cos pair into sincos();
    # -fno-trapping-math drops FP-exception bookkeeping. Neither changes any computed value.
    # -fno-math-errno: sqrt() becomes a single instruction instead of a call-with-errno fallback; no result changes
    target_compile_options(charqui PRIVATE -Wall -Wextra -Wpedantic -fno-math-errno)
endif()

if(CHARQUI_INSTALL)
    include(GNUInstallDirs)
    include(CMakePackageConfigHelpers)
    install(TARGETS charqui EXPORT charquiTargets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    )
    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
    install(EXPORT charquiTargets NAMESPACE charqui:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/charqui)

    # find_package(charqui) support for plain CMake and for ROS 2 packages built with colcon
    configure_package_config_file(cmake/charquiConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/charquiConfig.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/charqui
    )
    write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/charquiConfigVersion.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
    )
    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/charquiConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/charquiConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/charqui
    )
endif()

if(CHARQUI_BUILD_TESTS OR CHARQUI_BUILD_BENCHMARK)
    include(FetchContent)
endif()

if(CHARQUI_BUILD_TESTS)
    FetchContent_Declare(doctest
        GIT_REPOSITORY https://github.com/doctest/doctest.git
        GIT_TAG v2.4.11
        GIT_SHALLOW TRUE
    )
    set(DOCTEST_WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(DOCTEST_NO_INSTALL ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(doctest)
    enable_testing()
    add_subdirectory(tests)
endif()

if(CHARQUI_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

if(CHARQUI_BUILD_BENCHMARK)
    add_subdirectory(bench)
endif()

if(CHARQUI_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# If ament_cmake is available (ROS 2 / colcon), register the package in the ament index.
# The exported CMake config above is what downstream packages use in both worlds.
if(ament_cmake_FOUND)
    message(STATUS "ament_cmake found: exporting charqui as a ROS 2 package")
    ament_package()
endif()
