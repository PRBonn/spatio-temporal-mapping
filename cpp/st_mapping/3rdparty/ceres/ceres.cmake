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
set(USE_CUDA OFF CACHE BOOL "If use CUDA with ceres-solver")
set(BUILD_TESTING OFF CACHE BOOL "If build or not ceres-solver tests")
set(BUILD_EXAMPLES OFF CACHE BOOL "If build or not ceres-solver examples")
set(BUILD_BENCHMARKS OFF CACHE BOOL "If build or not ceres-solver benchmarks")
set(BUILD_DOCUMENTATION OFF CACHE BOOL "If build or not ceres-solver documentation")
set(MINIGLOG ON CACHE BOOL "Avoid using glog if not installed")

include(FetchContent)
FetchContent_Declare(ceres 
                     GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git 
                     GIT_TAG 2.2.0)
FetchContent_MakeAvailable(ceres)

