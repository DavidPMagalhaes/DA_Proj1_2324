// Edge.cpp
#include "Edge.h"

Edge::Edge(Vertex* source, Vertex* target, double capacity, bool isDirected, EdgeType type)
        : source(source), target(target), capacity(capacity), isDirected(isDirected), type(type) {}

Vertex* Edge::getSource() const {
    return source;
}

Vertex* Edge::getTarget() const {
    return target;
}

double Edge::getCapacity() const {
    return capacity;
}

bool Edge::getIsDirected() const {
    return isDirected;
}

EdgeType Edge::getType() const {
    return type;
}

