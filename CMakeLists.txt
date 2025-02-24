# Minimum CMake version required
cmake_minimum_required(VERSION 3.10)

# Project name
project(MatheuristicsCPPLC)

add_compile_definitions(IL_STD)

# Add include directory for headers
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories("C:\\Program Files\\IBM\\ILOG\\CPLEX_Studio1210\\cplex\\include")
include_directories("C:\\Program Files\\IBM\\ILOG\\CPLEX_Studio1210\\concert\\include")

# Add link libraries
link_directories("C:\\Program Files\\IBM\\ILOG\\CPLEX_Studio1210\\cplex\\lib\\x64_windows_msvc14\\stat_mda")
link_directories("C:\\Program Files\\IBM\\ILOG\\CPLEX_Studio1210\\concert\\lib\\x64_windows_msvc14\\stat_mda")

# Add the executable
add_executable(main main.cpp)
target_link_libraries(main concert ilocplex cplex12100)

# (Optional) Set C++ standard
set_target_properties(main PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES)