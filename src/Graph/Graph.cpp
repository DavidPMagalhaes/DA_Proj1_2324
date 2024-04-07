/**
 * @file Graph.cpp
 * @brief Implementation of the Graph class
 */
#include "Graph.h"
/**
 * @brief Constructs a Graph object.
 */
Graph::Graph() {}
/**
 * @brief Adds a vertex to the graph.
 * @param vertex Reference to the vertex to be added
 */
void Graph::addVertex(const Vertex& vertex) {
    vertices.push_back(vertex);
}
/**
 * @brief Adds an edge to the graph.
 * @param edge Reference to the edge to be added
 */
void Graph::addEdge(const Edge& edge) {
    edges.push_back(edge);
}
/**
 * @brief Forms a network from the given data.
 * @param reservoirsData Data for reservoirs
 * @param stationsData Data for pumping stations
 * @param citiesData Data for delivery sites (cities)
 * @param pipesData Data for pipelines
 */
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
/**
 * @brief Checks if the graph is connected.
 * @return True if the graph is connected, false otherwise
 */
bool Graph::isConnected() {
    if (vertices.empty()) {
        return false;
    }

    std::unordered_set<std::string> visited;
    bfs(&vertices[0], visited);

    return visited.size() == vertices.size();
}
/**
 * @brief Connects disconnected components of the graph.
 */
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
/**
 * @brief Breadth-first search starting from a given vertex.
 * @param start Pointer to the starting vertex
 * @param visited Set of visited vertex IDs
 */
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
