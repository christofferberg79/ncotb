cmake_minimum_required(VERSION 3.25)
project(ncotb LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# --- raylib (window, input, 3D drawing) ---
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 6.0
  GIT_SHALLOW TRUE
  SYSTEM)

# --- Dear ImGui (no CMake of its own) ---
FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.92.9b
  GIT_SHALLOW TRUE)

# --- rlImGui (raylib backend for ImGui) ---
FetchContent_Declare(rlimgui
  GIT_REPOSITORY https://github.com/raylib-extras/rlImGui.git
  GIT_TAG 1550009359ad975927f7f0e4a3f47e3f27123ea9)

# --- doctest ---
FetchContent_Declare(doctest
  GIT_REPOSITORY https://github.com/doctest/doctest.git
  GIT_TAG v2.5.3
  GIT_SHALLOW TRUE
  SYSTEM)

# imgui and rlImGui have no CMakeLists.txt, so MakeAvailable only downloads them;
# the imgui target below builds them.
FetchContent_MakeAvailable(raylib doctest imgui rlimgui)
# Warnings in third-party code aren't ours to fix, and VS Code's problem matcher
# would report them as task errors.
target_compile_options(raylib PRIVATE -w)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
# SYSTEM: warnings from third-party headers don't count against our code.
target_include_directories(imgui SYSTEM PUBLIC ${imgui_SOURCE_DIR} ${rlimgui_SOURCE_DIR})
target_link_libraries(imgui PUBLIC raylib)
target_compile_options(imgui PRIVATE -w)

# The same strict warnings for all our code (physics, app, tests), as errors,
# so problems fail the build instead of scrolling past.
option(NCOTB_WERROR "Treat warnings in our code as errors" ON)
function(ncotb_warnings target)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow
                         $<$<BOOL:${NCOTB_WERROR}>:-Werror>)
endfunction()

# AddressSanitizer + UndefinedBehaviorSanitizer for our own code (not the dependencies above).
option(NCOTB_SANITIZE "Build physics, app and tests with ASan + UBSan" OFF)
if(NCOTB_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-sanitize-recover=undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_subdirectory(physics)
add_subdirectory(app)

enable_testing()
add_subdirectory(tests)

# Formatting check: fails if any of our sources differs from .clang-format.
find_program(CLANG_FORMAT clang-format HINTS /Library/Developer/CommandLineTools/usr/bin)
if(CLANG_FORMAT)
  file(GLOB_RECURSE NCOTB_SOURCES CONFIGURE_DEPENDS
       ${PROJECT_SOURCE_DIR}/physics/*.h ${PROJECT_SOURCE_DIR}/physics/*.cpp
       ${PROJECT_SOURCE_DIR}/app/*.h ${PROJECT_SOURCE_DIR}/app/*.cpp
       ${PROJECT_SOURCE_DIR}/tests/*.h ${PROJECT_SOURCE_DIR}/tests/*.cpp)
  add_test(NAME format COMMAND ${CLANG_FORMAT} --dry-run --Werror ${NCOTB_SOURCES})
else()
  message(STATUS "clang-format not found: no formatting check")
endif()
