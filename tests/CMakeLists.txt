add_executable(physics_tests test_main.cpp test_math.cpp test_rigid_body.cpp test_integrators.cpp test_collision.cpp test_contacts.cpp test_friction.cpp test_joints.cpp test_interaction.cpp test_terrain.cpp test_vehicle.cpp)
target_link_libraries(physics_tests PRIVATE physics doctest::doctest)
ncotb_warnings(physics_tests)
add_test(NAME physics_tests COMMAND physics_tests)
