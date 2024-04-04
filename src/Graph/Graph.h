// Graph.h
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include "Edge.h"
#include "Vertex.h"

class Graph {
private:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;

public:
    Graph();

    // Add a vertex to the graph
    void addVertex(const Vertex& vertex);

    // Add an edge to the graph
    void addEdge(const Edge& edge);

    // Form the network from parsed input data
    void formNetwork(const std::vector<std::vector<std::string>>& reservoirsData,
                     const std::vector<std::vector<std::string>>& stationsData,
                     const std::vector<std::vector<std::string>>& citiesData,
                     const std::vector<std::vector<std::string>>& pipesData);
};

#endif // GRAPH_H
