# Python bindings (nanobind) for charqui. Added via add_subdirectory(python) from the top-level CMakeLists when
# CHARQUI_BUILD_PYTHON is ON, so the charqui C++ target already exists.
#
# Three build contexts share this file:
#   * scikit-build-core (pip wheel):  SKBUILD is defined, the package is installed at the wheel root
#   * colcon / ament (ROS 2):          ament_cmake_FOUND, installed into the prefix's site-packages so colcon's
#                                      PYTHONPATH hook picks it up; the pytest suite is registered with ament
#   * plain CMake into a prefix:       installed into the active interpreter's site-packages (pixi/conda/venv)

if(ament_cmake_FOUND)
    set(CHARQUI_AMENT_BUILD TRUE)
    message(STATUS "charqui bindings: building with ament_cmake")
    find_package(ament_cmake_python REQUIRED)
else()
    set(CHARQUI_AMENT_BUILD FALSE)
endif()

find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)

# Resolve nanobind from the active Python interpreter (pip, pixi, or the wheel build environment) unless the
# caller points at a specific installation.
if(NOT nanobind_DIR AND NOT nanobind_ROOT)
    execute_process(
        COMMAND ${Python_EXECUTABLE} -m nanobind --cmake_dir
        OUTPUT_VARIABLE nanobind_ROOT OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE nanobind_lookup
    )
endif()
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(charqui_core NB_STATIC LTO src/bindings.cpp)
set_target_properties(charqui_core PROPERTIES OUTPUT_NAME _core LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/charqui)
target_link_libraries(charqui_core PRIVATE charqui::charqui)

# Package layout in the build tree: used by the stub generator and importable for in-tree testing
configure_file(charqui/__init__.py ${CMAKE_CURRENT_BINARY_DIR}/charqui/__init__.py COPYONLY)

# Type stubs are generated automatically from the compiled module and shipped next to it (with py.typed)
option(CHARQUI_PYTHON_STUBS "Generate .pyi type stubs for the Python bindings" ON)
if(CHARQUI_PYTHON_STUBS AND NOT COMMAND nanobind_add_stub)
    message(WARNING "charqui bindings: this nanobind version has no nanobind_add_stub, skipping stub generation")
    set(CHARQUI_PYTHON_STUBS OFF)
endif()
set(charqui_python_files ${CMAKE_CURRENT_BINARY_DIR}/charqui/__init__.py)
if(CHARQUI_PYTHON_STUBS)
    nanobind_add_stub(charqui_core_stub
        MODULE charqui._core
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/charqui/_core.pyi
        PYTHON_PATH ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS charqui_core
        MARKER_FILE ${CMAKE_CURRENT_BINARY_DIR}/charqui/py.typed
    )
    list(APPEND charqui_python_files ${CMAKE_CURRENT_BINARY_DIR}/charqui/_core.pyi ${CMAKE_CURRENT_BINARY_DIR}/charqui/py.typed)
endif()

# Install destination of the `charqui` Python package, per build context
if(DEFINED SKBUILD)
    set(CHARQUI_PYTHON_DESTINATION ".")  # scikit-build-core: wheel root
elseif(CHARQUI_AMENT_BUILD)
    # colcon's environment hook adds <prefix>/lib/pythonX.Y/site-packages to PYTHONPATH. On Debian-based ROS,
    # ament reports lib/python3/dist-packages, which that hook does not cover, so map it to site-packages.
    ament_get_python_install_dir(charqui_ament_python_dir)
    if(charqui_ament_python_dir MATCHES "dist-packages")
        set(CHARQUI_PYTHON_DESTINATION "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
    else()
        set(CHARQUI_PYTHON_DESTINATION "${charqui_ament_python_dir}")
    endif()
else()
    file(TO_CMAKE_PATH "${Python_SITEARCH}" CHARQUI_PYTHON_DESTINATION)  # the active interpreter's site-packages
endif()
message(STATUS "charqui bindings: Python package installs to ${CHARQUI_PYTHON_DESTINATION}/charqui")

install(TARGETS charqui_core LIBRARY DESTINATION ${CHARQUI_PYTHON_DESTINATION}/charqui)
install(FILES ${charqui_python_files} DESTINATION ${CHARQUI_PYTHON_DESTINATION}/charqui)

# Under colcon, run the pytest suite against the installed package
if(CHARQUI_AMENT_BUILD AND CHARQUI_BUILD_TESTS)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(charqui_python_tests ${PROJECT_SOURCE_DIR}/tests/python TIMEOUT 300)
endif()
