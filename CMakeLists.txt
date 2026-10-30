cmake_minimum_required(VERSION 3.15)
project(CrashTest CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# static MSVC runtime -> the exe runs on any Windows without installing redistributables
if(MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

include(FetchContent)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
FetchContent_Declare(raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.5
  GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(raylib)

add_executable(CrashTest src/main.cpp src/car_sim.h)
target_link_libraries(CrashTest PRIVATE raylib)
if(MSVC)
  target_compile_options(CrashTest PRIVATE /utf-8 /O2)
  set_target_properties(CrashTest PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
endif()

# copy the font next to the exe
add_custom_command(TARGET CrashTest POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/assets $<TARGET_FILE_DIR:CrashTest>/assets)

# headless physics test (no window needed):  cmake --build build --target physics_test
add_executable(physics_test EXCLUDE_FROM_ALL tools/physics_test.cpp)
