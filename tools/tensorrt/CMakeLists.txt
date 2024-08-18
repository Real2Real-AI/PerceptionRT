cmake_minimum_required(VERSION 3.5)
project(centerpoint_pybind LANGUAGES CXX CUDA)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# CUDA Flag settings
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_61,code=sm_61")
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_80,code=sm_80") # A100
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_89,code=sm_89") # RTX4090
#set(PYTHON_INCLUDE_DIR "/usr/include/python3.8")
#set(PYTHON_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")

# Find dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(pybind11 REQUIRED)
find_package(CUDA REQUIRED)
find_package(TensorRT REQUIRED)
find_package(YAML-CPP REQUIRED)
#find_package(PythonLibs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

function(copy_output target_name to_path)
  add_custom_command(TARGET ${target_name} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
      "$<TARGET_FILE:${target_name}>"
      "${PROJECT_SOURCE_DIR}/${to_path}/$<TARGET_FILE_NAME:${target_name}>"
  )
endfunction()

include_directories(
  include
  ../ros2/centerpoint/include
  ${Python3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

set(THIRD_PARTY_LIBS
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${TensorRT_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  stdc++fs
)

#add_subdirectory(../ros2/centerpoint/src ${CMAKE_BINARY_DIR}/build)

# Add the centerpoint source files directly
set(CENTERPOINT_SRCS
  ../ros2/centerpoint/src/centerpoint/centerpoint.cc
  ../ros2/centerpoint/src/centerpoint/network.cc
  ../ros2/centerpoint/src/centerpoint/voxelization.cu
  ../ros2/centerpoint/src/centerpoint/voxelization.cc
  ../ros2/centerpoint/src/centerpoint/postprocess.cu
  ../ros2/centerpoint/src/centerpoint/postprocess.cc
  ../ros2/centerpoint/src/pillarscatter/pillarscatter-kernel.cu
  ../ros2/centerpoint/src/pillarscatter/pillarscatter.cc
)

# Create the pycenterpoint module and add centerpoint source files directly
pybind11_add_module(pycenterpoint MODULE
  module.cc
  src/centerpoint/wrapper.cc
  ${CENTERPOINT_SRCS} # Add centerpoint source files directly to pycenterpoint
)

target_link_libraries(pycenterpoint PRIVATE
  ${THIRD_PARTY_LIBS}
  Python3::Python
)

#copy_output(pycenterpoint ../../object_detection)
copy_output(pycenterpoint ./)