cmake_minimum_required(VERSION 4.0)
project(Robot_ARM_Piper)

set(CMAKE_CXX_STANDARD 17)

include_directories(.)
include_directories(mujoco_cpp)
include_directories("E:/Mujoco/mujoco-3.3.0-windows-x86_64/include")

link_directories("E:/Mujoco/mujoco-3.3.0-windows-x86_64/bin")

add_executable(Robot_ARM_Piper
        mujoco_cpp/main.cpp)
