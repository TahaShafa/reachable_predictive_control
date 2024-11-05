cmake_minimum_required(VERSION 3.10)

# Set CMake policy CMP0167 to use new Boost handling
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# Set the project name and version
project(RPC VERSION 1.0)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Add the executable
add_executable(RPC src/main.cpp src/uav/uav_dynamics.cpp src/grs/grs.cpp src/synthesis/controller_synthesis.cpp src/solve_ode/solve_ode.cpp)

# Link the libraries
target_link_libraries(RPC 
    ${Boost_LIBRARIES}        # Link Boost libraries
    glpk                      # Link GLPK library
)

# Link Eigen (Eigen is header-only, no linking required)
target_include_directories(RPC PRIVATE ${EIGEN3_INCLUDE_DIR})
