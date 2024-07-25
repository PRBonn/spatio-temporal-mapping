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
project(st_mapping_cpp VERSION 0.1.0 LANGUAGES CXX)

# Setup build options
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_SOPHUS "Use system pre-installed Sophus" OFF)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(USE_SYSTEM_TSL-ROBIN-MAP "Use system pre-installed tsl_robin" OFF)
option(USE_SYSTEM_OPENCV "Use system pre-installed OpenCV" ON)
option(USE_SYSTEM_CERES "Use system pre-installed ceres-solver" ON)

# Set build type
set(CMAKE_BUILD_TYPE Release)

# cmake options and functions
include(cmake/StandardProjectOptions.cmake)
include(cmake/GlobalTargetProperty.cmake)

# Find dependencies
include(3rdparty/find_dependencies.cmake)

# Add all the libraries (and executables)
add_subdirectory(core)
add_subdirectory(metrics)
