# MIT License
#
# Copyright (c) 2024 Luca Lobefaro, Meher V.R. Malladi, Tiziano Guadagnino,
# Cyrill Stachniss
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

add_library(utils 
  ${CMAKE_CURRENT_SOURCE_DIR}/voxel_hash_map/VoxelHashMap.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/decorators/ExecutionTimerDecorator.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/parser/PATHoBotDatasetParser.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/config/Config.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/superpoint/superpoint.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/superpoint/superpoint_matcher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/visualization/GlasshouseVisualizer.cpp)
target_compile_features(utils PRIVATE cxx_std_20)

# Dependencies
target_include_directories(utils PUBLIC ${EIGEN3_INCLUDE_DIR} ${OpenCV_INCLUDE_LIBS})
target_link_libraries(utils PUBLIC ${Eigen3} ${OpenCV_LIBS} yaml-cpp Open3D::Open3D JsonCpp::JsonCpp)

# Torch
target_link_libraries(utils PRIVATE torch)
target_link_options(utils PRIVATE ${TORCH_CXX_FLAGS})

