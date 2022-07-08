cmake_minimum_required(VERSION 3.0.2)
project(se3track)

add_compile_options(-fPIC)
if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
    message(STATUS "optional:-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

include_directories("./include")
include_directories("./src")

include_directories(
  /usr/include/eigen3
)


# 进行接口文件测试
# link_directories("./build/lib")

# 用户的测试例
add_executable(
  se2_interpolation
  src/se2_interpolation.cpp
) 

