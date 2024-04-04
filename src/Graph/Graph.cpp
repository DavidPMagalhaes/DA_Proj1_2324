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
    // Create vertices for reservoirs
    for (const auto& data : reservoirsData) {
        Vertex vertex(data[3], VertexType::RESERVOIR); // Assuming index 3 contains the code
        addVertex(vertex);
    }

    // Create vertices for pumping stations
    for (const auto& data : stationsData) {
        Vertex vertex(data[1], VertexType::PUMPING_STATION); // Assuming index 1 contains the code
        addVertex(vertex);
    }

    // Create vertices for delivery sites (cities)
    for (const auto& data : citiesData) {
        Vertex vertex(data[2], VertexType::DELIVERY_SITE); // Assuming index 2 contains the code
        addVertex(vertex);
    }

    // Create edges for pipelines
    for (const auto& data : pipesData) {
        // Assuming the source and target service points are identified by their codes
        Vertex* source = nullptr;
        Vertex* target = nullptr;
        for (auto& vertex : vertices) {
            if (vertex.getId() == data[0]) { // Assuming index 0 contains the code of source service point
                source = &vertex;
            }
            if (vertex.getId() == data[1]) { // Assuming index 1 contains the code of target service point
                target = &vertex;
            }
            if (source && target) {
                break;
            }
        }
        if (source && target) {
            Edge edge(source, target, std::stod(data[2]), true, EdgeType::PIPELINE); // Assuming index 2 contains the capacity
            addEdge(edge);
        }
    }
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

    // Create a set to keep track of visited vertices
    std::unordered_set<std::string> visited;

    // Traverse all vertices
    for (Vertex& vertex : vertices) {
        if (visited.find(vertex.getId()) == visited.end()) {
            // If the vertex hasn't been visited yet, perform BFS traversal from it
            bfs(&vertex, visited);
        }
    }

    // After traversal, if the graph is still not connected, connect disconnected components
    if (!isConnected()) {
        // Connect the first vertex of each disconnected component to the last vertex of the previous component
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

        // Traverse all adjacent vertices
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
