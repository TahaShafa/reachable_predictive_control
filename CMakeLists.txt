cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
    if($ENV{pybullet_env})
      set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type (Debug or Release)" FORCE)
    else()
      set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type (Debug or Release)" FORCE)
    endif()
endif()

message(STATUS "CMake build type: ${CMAKE_BUILD_TYPE}")

# Set CMake policy CMP0167 to use new Boost handling
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Ignore warning for pybind
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 NEW)
endif()

# Set the project name and version
project(RPC VERSION 1.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Ensure Python 3.9 is used
set(Python3_ROOT_DIR "/home/taha-shafa/miniconda3/envs/pybullet_env")
set(Python3_EXECUTABLE "/home/taha-shafa/miniconda3/envs/pybullet_env/bin/python3")
find_package(Python3 3.9 EXACT REQUIRED COMPONENTS Interpreter Development)

# Find pybind11 in the correct environment
find_package(pybind11 REQUIRED HINTS /home/taha-shafa/miniconda3/envs/pybullet_env/lib/python3.9/site-packages/pybind11/share/cmake/pybind11)

# Find Eigen and Boost packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost 1.47 REQUIRED COMPONENTS system filesystem iostreams)

# Manually add GLPK include and library paths
include_directories(/usr/include) # GLPK headers
link_directories(/usr/lib/x86_64-linux-gnu) # GLPK libraries

# Specify the include directory for Eigen, Boost, and gnuplot-iostream
include_directories(
    ${CMAKE_SOURCE_DIR}/include 
    ${EIGEN3_INCLUDE_DIR} 
    ${Boost_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/third_party/gnuplot-iostream
)

# Add a reusable library for shared code
add_library(rpc_core STATIC
    src/uav/uav_dynamics.cpp
    src/grs/grs.cpp
    src/synthesis/controller_synthesis.cpp
    src/solve_ode/solve_ode.cpp
    src/car/car_dynamics.cpp
)
set_target_properties(rpc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Add the executable
add_executable(RPC
  src/main.cpp
)

# Link the reusable library and other libraries to the executable
target_link_libraries(RPC 
    rpc_core               # Link shared library
    ${Boost_LIBRARIES}     # Link Boost libraries
    glpk                   # Link GLPK library
)

# Add the Python module in the pybullet directory
add_subdirectory(pybullet)

