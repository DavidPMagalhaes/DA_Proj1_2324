// Vertex.h
#ifndef VERTEX_H
#define VERTEX_H

#include <string>
#include <vector>
#include "Edge.h"

enum class VertexType {
    RESERVOIR,
    PUMPING_STATION,
    DELIVERY_SITE
};

class Edge;

class Vertex {
private:
    std::string id;
    VertexType type;
    std::vector<Edge*> edges;

public:
    Vertex(const std::string& id, VertexType type);
    void addEdge(Edge* edge);
    const std::string& getId() const;
    VertexType getType() const;
};

#endif // VERTEX_H
