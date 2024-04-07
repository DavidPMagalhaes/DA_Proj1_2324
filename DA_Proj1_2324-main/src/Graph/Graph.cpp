// Graph.cpp
#include "Graph.h"

Graph::Graph() {}

void Graph::addVertex(const Vertex& vertex) {
    vertices.push_back(vertex);
}

void Graph::addEdge(const Edge& edge) {
    edges.push_back(edge);
}

void Graph::formNetwork(const std::vector<std::vector<std::string>>& reservoirsData,
                        const std::vector<std::vector<std::string>>& stationsData,
                        const std::vector<std::vector<std::string>>& citiesData,
                        const std::vector<std::vector<std::string>>& pipesData) {
    // Implementation of formNetwork
}

bool Graph::isConnected() {
    if (vertices.empty()) {
        return false;
    }

    std::unordered_set<std::string> visited;
    bfs(&vertices[0], visited);

    return visited.size() == vertices.size();
}

void Graph::connectDisconnectedComponents() {
    if (isConnected()) {
        return; // Graph is already connected
    }

    std::unordered_set<std::string> visited;
    for (Vertex& vertex : vertices) {
        if (visited.find(vertex.getId()) == visited.end()) {
            bfs(&vertex, visited);
        }
    }

    if (!isConnected()) {
        for (size_t i = 1; i < vertices.size(); ++i) {
            addEdge(Edge(&vertices[i - 1], &vertices[i], 0, true, EdgeType::PIPELINE));
        }
    }
}

void Graph::bfs(Vertex* start, std::unordered_set<std::string>& visited) {
    std::queue<Vertex*> queue;
    queue.push(start);
    visited.insert(start->getId());

    while (!queue.empty()) {
        Vertex* current = queue.front();
        queue.pop();

        for (const Edge& edge : edges) {
            if (edge.getSource() == current) {
                Vertex* neighbor = edge.getTarget();
                if (visited.find(neighbor->getId()) == visited.end()) {
                    queue.push(neighbor);
                    visited.insert(neighbor->getId());
                }
            }
        }
    }
}

