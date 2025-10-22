cmake_minimum_required(VERSION 4.0)
project(Robot_ARM_Piper)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(mujoco_cpp)

add_executable(Robot_ARM_Piper
        mujoco_cpp/main.cpp)
