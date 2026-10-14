add_executable(charqui-tests
    test_main.cpp
    test_roots.cpp
    test_target.cpp
    test_random.cpp
    test_waypoints.cpp
)
target_link_libraries(charqui-tests PRIVATE charqui::charqui doctest::doctest)
target_compile_options(charqui-tests PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wno-missing-field-initializers>)
add_test(NAME charqui-tests COMMAND charqui-tests)
