/**
 * @file Edge.cpp
 * @brief Implementation of the Edge class
 */
#include "Edge.h"

/**
 * @brief Constructs an Edge object with given source, target, capacity, and directedness.
 * @param source Pointer to the source vertex of the edge
 * @param target Pointer to the target vertex of the edge
 * @param capacity Capacity of the edge
 * @param isDirected Boolean indicating whether the edge is directed or not
 */
Edge::Edge(Vertex* source, Vertex* target, double capacity, bool isDirected)
        : source(source), target(target), capacity(capacity), isDirected(isDirected) {}

/**
 * @brief Getter method for the source vertex of the edge.
 * @return Pointer to the source vertex
 */
Vertex* Edge::getSource() const {
    return source;
}
/**
 * @brief Getter method for the target vertex of the edge.
 * @return Pointer to the target vertex
 */
Vertex* Edge::getTarget() const {
    return target;
}
/**
 * @brief Getter method for the capacity of the edge.
 * @return Capacity of the edge
 */
double Edge::getCapacity() const {
    return capacity;
}
/**
 * @brief Getter method for the directedness of the edge.
 * @return True if the edge is directed, false otherwise
 */
bool Edge::getIsDirected() const {
    return isDirected;
}
