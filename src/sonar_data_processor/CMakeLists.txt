cmake_minimum_required(VERSION 3.5...3.26)
project(sonar_data_processor LANGUAGES CXX)

add_subdirectory(pybind11)
pybind11_add_module(sonar_data_processor sonar_data_processor.cpp)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(sonar_data_processor PRIVATE ${PYTHON_LIBRARIES})