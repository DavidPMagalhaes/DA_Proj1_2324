#ifndef CAL_PROJ_VERTEX_H
#define CAL_PROJ_VERTEX_H

#include "Graph.h"

#include "Edge.h"
#include "MutablePriorityQueue.h"

using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

/**
 * @brief A template class representing a vertex in a graph.
 *
 * This class represents a vertex in a graph. It stores information about the
 * vertex's content, outgoing edges, shortest distance to the source vertex in a
 * path-finding algorithm, and predecessor vertex in that path.
 *
 * @tparam T The type of the vertex's content.
 */
template <class T>
class Vertex {
    T info;						// content of the vertex
    // outgoing edges

    double dist = 0;
    Vertex<T> *path = NULL;
    int queueIndex = 0; 		// required by MutablePriorityQueue

    bool visited = false;		// auxiliary field
    bool processing = false;	// auxiliary field

    /**
 * @brief A template class representing a vertex in a graph.
 *
 * This class represents a vertex in a graph. It stores information about the
 * vertex's content, outgoing edges, shortest distance to the source vertex in a
 * path-finding algorithm, and predecessor vertex in that path.
 *
 * @tparam T The type of the vertex's content.
 */
    void addEdge(Vertex<T> *dest, double w);
public:
    /**
     * @brief Constructs a new Vertex object.
     *
     * This constructor creates a new Vertex object with the given content.
     *
     * @param in The content of the vertex.
     */
    Vertex(T in);

    /**
     * @brief Gets the content of the vertex.
     *
     * This function returns the content of the vertex.
     *
     * @return The content of the vertex.
     */
    T getInfo() const;

    /**
     * @brief Gets the shortest distance to the source vertex.
     *
     * This function returns the shortest distance to the source vertex in a
     * path-finding algorithm.
     *
     * @return The shortest distance to the source vertex.
     */
    double getDist() const;

    /**
     * @brief Gets the predecessor vertex in the shortest path.
     *
     * This function returns the predecessor vertex in the shortest path to the
     * source vertex in a path-finding algorithm.
     *
     * @return The predecessor vertex in the shortest path.
     */
    Vertex *getPath() const;

    /**
     * @brief Compares two vertices based on their distances to the source vertex.
     *
     * This operator is required by the MutablePriorityQueue class, which uses
     * it to determine the order of vertices in the priority queue.
     *
     * @param vertex The vertex to compare to.
     * @return True if this vertex's distance is less than the other vertex's distance.
     */
    bool operator<(Vertex<T> & vertex) const; // // required by MutablePriorityQueue
    friend class Graph<T>;
    friend class MutablePriorityQueue<Vertex<T>>;

    std::vector<Edge<T> > adj;

    vector<Vertex<T> *> getAdj() const;

    double getWeight(Vertex<T> *dest);
};


template <class T>
Vertex<T>::Vertex(T in): info(in) {}

template <class T>
void Vertex<T>::addEdge(Vertex<T> *d, double w) {
    adj.push_back(Edge<T>(d, w));
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template <class T>
Vertex<T> *Vertex<T>::getPath() const {
    return this->path;
}

template <class T>
vector<Vertex<T>*> Vertex<T>::getAdj() const {
    vector<Vertex<T>*> adjVertices;
    for (auto e : adj)
        adjVertices.push_back(e.dest);
    return adjVertices;
}

template <class T>
double Vertex<T>::getWeight(Vertex<T>* dest) {
    for (auto &e: adj) {
        if (e.dest == dest) {
            return e.weight;
        }
    }
    return -1; // or any other value that indicates that the edge was not found
}


#endif //CAL_PROJ_VERTEX_H