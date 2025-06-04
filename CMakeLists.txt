cmake_minimum_required(VERSION 3.14)
project(5DOF-Arm-simulator)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent) 

# Fetch Eigen3
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
)
FetchContent_MakeAvailable(eigen)

# Fetch raylib
FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG        4.5.0
)
FetchContent_MakeAvailable(raylib)

set(THIRD_PARTY_LIBS 
  Eigen3::Eigen
  raylib
)

file(GLOB_RECURSE SOURCE source/*.cpp)
add_executable(Arm_simulator ${SOURCE})
target_include_directories(Arm_simulator PRIVATE include)

target_link_libraries(Arm_simulator PRIVATE ${THIRD_PARTY_LIBS})
