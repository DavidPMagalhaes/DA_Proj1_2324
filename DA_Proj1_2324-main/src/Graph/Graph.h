// Graph.h
#ifndef GRAPH_H
#define GRAPH_H

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include "Edge.h"
#include "Vertex.h"

class Graph {
private:
    std::vector<Vertex> vertices;
    std::vector<Edge> edges;

public:
    Graph();

    void addVertex(const Vertex& vertex);
    void addEdge(const Edge& edge);
    void formNetwork(const std::vector<std::vector<std::string>>& reservoirsData,
                     const std::vector<std::vector<std::string>>& stationsData,
                     const std::vector<std::vector<std::string>>& citiesData,
                     const std::vector<std::vector<std::string>>& pipesData);
    bool isConnected();
    void connectDisconnectedComponents();
    void bfs(Vertex* start, std::unordered_set<std::string>& visited);
};

#endif // GRAPH_H

