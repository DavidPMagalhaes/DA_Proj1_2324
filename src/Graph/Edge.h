#ifndef CAL_PROJ_EDGE_H
#define CAL_PROJ_EDGE_H

#include "Graph.h"
#include "Vertex.h"

using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

/**
 * Edge class represents an edge in a graph
 *
 * @tparam T the type of elements stored in the vertex
 */
template <class T>
class Edge {
    double weight;  ///< the weight of the edge
public:
    Vertex<T> * dest;  ///< the destination vertex

    /**
     * Constructor for an Edge object.
     *
     * @param d the destination vertex
     * @param w the weight of the edge
     */
    Edge(Vertex<T> *d, double w);

    friend class Graph<T>;
    friend class Vertex<T>;
};

/**
 * Constructor for an Edge object.
 *
 * @tparam T the type of elements stored in the vertex
 * @param d the destination vertex
 * @param w the weight of the edge
 */
template <class T>
Edge<T>::Edge(Vertex<T> *d, double w): dest(d), weight(w) {}


#endif //CAL_PROJ_EDGE_H
