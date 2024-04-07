// Vertex.cpp
#include "Vertex.h"

// Constructor definition
Vertex::Vertex(const std::string& id) : id(id) {}

// Definition for other member functions
void Vertex::addEdge(Edge* edge) {
    edges.push_back(edge);
}

const std::string& Vertex::getId() const {
    return id;
}

VertexType Vertex::getType() const {
    return type;
}

