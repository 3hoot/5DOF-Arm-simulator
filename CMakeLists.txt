cmake_minimum_required(VERSION 3.14)
project(5DOF-Arm-simulator)

include(FetchContent) 
# Fetch spdlog
FetchContent_Declare(
  spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        v1.13.0
)
FetchContent_MakeAvailable(spdlog)

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
  spdlog::spdlog
  Eigen3::Eigen
  raylib
)

file(GLOB_RECURSE SOURCE source/*.cpp)
add_executable(Arm_simulator ${SOURCE})
target_include_directories(Arm_simulator PRIVATE include)

target_link_libraries(Arm_simulator PRIVATE ${THIRD_PARTY_LIBS})
