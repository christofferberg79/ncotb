add_executable(physics_tests test_main.cpp test_math.cpp test_rigid_body.cpp)
target_link_libraries(physics_tests PRIVATE physics doctest::doctest)
add_test(NAME physics_tests COMMAND physics_tests)
