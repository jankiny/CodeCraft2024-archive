cmake_minimum_required(VERSION 3.24)
project(CodeCraft2024)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable(CodeCraft2024 src/main.cpp)
