add_library(physics STATIC
  body/rigid_body.cpp
  collision/collide.cpp
  collision/raycast.cpp
  integration/integrator.cpp
  joints/joint.cpp
  solver/constraint_solver.cpp
  solver/lcp.cpp
  world.cpp)
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ncotb_warnings(physics)
