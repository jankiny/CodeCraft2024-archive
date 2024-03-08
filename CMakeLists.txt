cmake_minimum_required(VERSION 3.8)
project(CodeCraft2024)

if (!WIN32)
    link_libraries(pthread rt m)
endif (!WIN32)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXE_LINKER_FLAGS "-static")

add_executable(CodeCraft2024 src/main.cpp)
