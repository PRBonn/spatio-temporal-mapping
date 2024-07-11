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
# Mapping systems
add_executable(mapping Mapping.cpp)
target_compile_features(mapping PUBLIC cxx_std_20)
target_include_directories(mapping PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(mapping PRIVATE dataset core yaml-cpp)

add_executable(slam SLAMInPreviousMap.cpp)
target_compile_features(slam PUBLIC cxx_std_20)
target_include_directories(slam PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(slam PRIVATE dataset core yaml-cpp)

# My approaches
add_executable(sp_deform SuperPointDeform.cpp)
target_compile_features(sp_deform PUBLIC cxx_std_20)
target_include_directories(sp_deform PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_LIBS})
target_link_libraries(sp_deform PRIVATE dataset core yaml-cpp ${OpenCV_LIBS} cilantro_wrapper)

add_executable(associate ComputeAssociations.cpp)
target_compile_features(associate PUBLIC cxx_std_20)
target_include_directories(associate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/.. ${OpenCV_INCLUDE_LIBS})
target_link_libraries(associate PRIVATE dataset core yaml-cpp ${OpenCV_LIBS} cilantro_wrapper)


# Baselines
add_executable(cilantro_non_rigid_icp CilantroNonRigidICP.cpp)
target_compile_features(cilantro_non_rigid_icp PUBLIC cxx_std_17)
target_include_directories(cilantro_non_rigid_icp PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(cilantro_non_rigid_icp PRIVATE dataset core yaml-cpp cilantro_wrapper)


# Evaluation
add_executable(evaluate Evaluation.cpp)
target_compile_features(evaluate PUBLIC cxx_std_20)
target_include_directories(evaluate PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(evaluate PRIVATE dataset core yaml-cpp)
