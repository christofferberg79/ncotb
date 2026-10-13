add_library(physics STATIC
  body/heightfield.cpp
  body/rigid_body.cpp
  collision/collide.cpp
  collision/raycast.cpp
  collision/terrain.cpp
  integration/integrator.cpp
  io/snapshot.cpp
  joints/joint.cpp
  solver/constraint_solver.cpp
  solver/lcp.cpp
  vehicle/car.cpp
  world.cpp)
target_include_directories(physics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
ncotb_warnings(physics)
