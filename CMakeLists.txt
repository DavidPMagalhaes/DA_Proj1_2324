cmake_minimum_required(VERSION 3.21)
project(DA2223)

set(CMAKE_CXX_STANDARD 14)

include_directories(src)

add_executable(DA2223
        src/readFiles.cpp
        src/readFiles.h
        src/main.cpp)
