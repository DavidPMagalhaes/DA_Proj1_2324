// Edge.h
#ifndef EDGE_H
#define EDGE_H

#include "Vertex.h"

enum class EdgeType {
    PIPELINE,
    SUPPLY_ROUTE
};

class Vertex;

class Edge {
private:
    Vertex* source;
    Vertex* target;
    double capacity;
    bool isDirected;
    EdgeType type;

public:
    Edge(Vertex* source, Vertex* target, double capacity, bool isDirected, EdgeType type);
    Vertex* getSource() const;
    Vertex* getTarget() const;
    double getCapacity() const;
    bool getIsDirected() const;
    EdgeType getType() const;
};

#endif // EDGE_H
