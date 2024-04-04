// Vertex.cpp

#include "Vertex.h"

Vertex::Vertex(const std::string& id) : id(id) {}

void Vertex::addEdge(Edge* edge) {
    edges.push_back(edge);
}

const std::string& Vertex::getId() const {
    return id;
}
