/**
 * @file Vertex.cpp
 * @brief Implementation of the Vertex class
 */

#include "Vertex.h"

/**
 * @brief Constructs a Vertex object with given ID.
 * @param id The ID of the vertex
 */
Vertex::Vertex(const std::string& id) : id(id) {}

/**
 * @brief Adds an edge to the vertex.
 * @param edge Pointer to the edge to be added
 */
void Vertex::addEdge(Edge* edge) {
    edges.push_back(edge);
}

/**
 * @brief Getter method for the ID of the vertex.
 * @return Const reference to the ID of the vertex
 */
const std::string& Vertex::getId() const {
    return id;
}
