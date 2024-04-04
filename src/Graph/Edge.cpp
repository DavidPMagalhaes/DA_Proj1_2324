// Edge.cpp

#include "Edge.h"

Edge::Edge(Vertex* source, Vertex* target, double capacity, bool isDirected)
        : source(source), target(target), capacity(capacity), isDirected(isDirected) {}

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
