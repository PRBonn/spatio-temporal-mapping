# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino, Cyrill Stachniss
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 3.16...3.26)
project(st_mapping_pybind VERSION 0.1.0 LANGUAGES CXX)

# Set build type
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# "Import" pybind11
set(PYBIND11_NEWPYTHON ON)
find_package(pybind11 CONFIG REQUIRED)

# Import the cpp part
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/st_mapping/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/st_mapping ${CMAKE_CURRENT_BINARY_DIR}/st_mapping)
else()
  # TODO: fix the TAG at some point
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching ST-MAPPING v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_st_mapping_core PREFIX st_mapping_core
    GIT_REPOSITORY https://github.com/PRBonn/spatio-temporal-mapping.git 
    GIT_TAG origin/main
    SOURCE_SUBDIR cpp/st_mapping)
  FetchContent_MakeAvailable(ext_st_mapping_core)
endif()

# Import the pybind code
add_subdirectory(st_mapping/pybind)
