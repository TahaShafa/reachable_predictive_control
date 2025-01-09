# Create the Python module
pybind11_add_module(pybullet_module bindings.cpp)

# Link the reusable shared library and Boost libraries to the Python module
target_link_libraries(pybullet_module PRIVATE
    rpc_core               # Link the shared core library
    ${Boost_LIBRARIES}     # Link Boost libraries
    glpk                   # Link GLPK if it's used by rpc_core or bindings
)

# Include directories from the main project
target_include_directories(pybullet_module PRIVATE
    ${CMAKE_SOURCE_DIR}/include     # Main project include directory
    ${EIGEN3_INCLUDE_DIR}           # Eigen include directory
    ${Boost_INCLUDE_DIRS}           # Boost include directories
)

# Ensure Boost.IOStreams component is correctly specified
find_package(Boost 1.47 REQUIRED COMPONENTS system filesystem iostreams)

