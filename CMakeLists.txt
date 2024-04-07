cmake_minimum_required(VERSION 3.26)
project(DA_Proj1_2324_main__3_)

set(CMAKE_CXX_STANDARD 14)

include_directories(DA_Proj1_2324-main/src)
include_directories(DA_Proj1_2324-main/src/Graph)

add_executable(DA_Proj1_2324_main__3_
        .idea/.gitignore
        ".idea/DA_Proj1_2324-main (3).iml"
        .idea/modules.xml
        DA_Proj1_2324-main/.idea/.gitignore
        DA_Proj1_2324-main/.idea/DA_Proj1_2324.iml
        DA_Proj1_2324-main/.idea/modules.xml
        DA_Proj1_2324-main/.idea/vcs.xml
        DA_Proj1_2324-main/Project1DataSetSmall/Cities_Madeira.csv
        DA_Proj1_2324-main/Project1DataSetSmall/Map-Madeira.pdf
        DA_Proj1_2324-main/Project1DataSetSmall/Pipes_Madeira.csv
        DA_Proj1_2324-main/Project1DataSetSmall/Reservoirs_Madeira.csv
        DA_Proj1_2324-main/Project1DataSetSmall/Stations_Madeira.csv
        DA_Proj1_2324-main/src/Graph/Edge.cpp
        DA_Proj1_2324-main/src/Graph/Edge.h
        DA_Proj1_2324-main/src/Graph/Graph.cpp
        DA_Proj1_2324-main/src/Graph/Graph.h
        DA_Proj1_2324-main/src/Graph/MutablePriorityQueue.h
        DA_Proj1_2324-main/src/Graph/Vertex.cpp
        DA_Proj1_2324-main/src/Graph/Vertex.h
        DA_Proj1_2324-main/src/main.cpp
        DA_Proj1_2324-main/src/readFiles.cpp
        DA_Proj1_2324-main/src/readFiles.h
        DA_Proj1_2324-main/src/utils.cpp
        DA_Proj1_2324-main/src/utils.h
        DA_Proj1_2324-main/README.md)
