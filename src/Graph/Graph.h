#ifndef CAL_PROJ_GRAPH_H
#define CAL_PROJ_GRAPH_H


#include <list>
#include <algorithm>
#include <limits>
#include <cmath>
#include <vector>
#include <queue>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include "Edge.h"
#include "Vertex.h"
#include "MutablePriorityQueue.h"

using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

#define INF std::numeric_limits<double>::max()

/**
 * @brief A template class Graph which represents a graph of vertices connected by weighted edges.
 * @tparam T
 */
template <class T>
class Graph {
    // vertex set
    vector<Edge<T>*> edgeSet;
public:
    const vector<Edge<T> *> &getEdgeSet() const {
        return edgeSet;
    }

    std::vector<Vertex<T> *> vertexSet;
private:

    void dfsVisit(Vertex<T> *v,  std::vector<T> & res) const;

    bool dfsIsDAG(Vertex<T> *v) const;
public:
    /**
     * @brief Finds a vertex with a given info value.
     *
     * @param in - the value to search for.
     * @return A pointer to the vertex containing the value, or NULL if it's not found.
     */
    Vertex<T> *findVertex(const T &in) const;
    /**
     * @brief Finds a vertex with a given tag.
     *
     * @param tag - the tag to search for.
     * @return A pointer to the vertex containing the tag, or NULL if it's not found.
     */
    Vertex<T> *findVertexTag(string tag) const;
    /**
     * @brief Gets the number of vertices in the graph.
     *
     * @return The number of vertices in the graph.
     */
    int getNumVertex() const;
    /**
     * @brief Adds a vertex with a given value to the graph.
     *
     * @param in - the value to add.
     * @return True if the vertex was successfully added, false if it already existed.
     */
    bool addVertex(const T &in);
    /**
     * @brief Removes a vertex with a given value from the graph.
     *
     * @param in - the value to remove.
     * @return True if the vertex was successfully removed, false if it didn't exist.
     */
    bool removeVertex(const T &in);
    /**
     * @brief Adds an edge between two vertices with given values.
     *
     * @param sourc - the value of the source vertex.
     * @param dest - the value of the destination vertex.
     * @param w - the weight of the edge.
     * @return True if the edge was successfully added, false if either of the vertices didn't exist.
     */
    bool addEdge(const T &sourc, const T &dest, double w);
    /**
     * @brief Removes the edge between two vertices with given values.
     *
     * @param sourc - the value of the source vertex.
     * @param dest - the value of the destination vertex.
     * @return True if the edge was successfully removed, false if it didn't exist.
     */
    bool removeEdge(const T &sourc, const T &dest);
    /**
     * @brief Performs a Depth-First Search on the graph.
     *
     * @return A vector containing the values of the vertices in the order they were visited.
     */
    std::vector<T> dfs() const;
    vector<T> dfsPersussion(const T &orig, const T &dest) const;
    /**
     * @brief Performs a Breadth-First Search on the graph, starting from a given vertex.
     *
     * @param source - the value of the source vertex.
     * @return A vector containing the values of the vertices in the order they were visited.
     */
    std::vector<T> bfs(const T &source) const;
    /**
     * @brief Performs a Topological Sort on the graph.
     *
     * @return A vector containing the values of the vertices in the order they were sorted.
     */
    std::vector<T> topsort() const;
    /**
     * @brief Finds the maximum number of children of a given vertex in a spanning tree of the graph.
     *
     * @param source - the value of the source vertex.
     * @param inf - a reference to a variable to store the vertex with the maximum number of children.
     * @return The maximum number of children.
     */
    int maxNewChildren(const T &source, T &inf) const;
    /**
     * @brief Checks if the graph is a Directed Acyclic Graph (DAG).
     *
     * @return true if the graph is a DAG, false otherwise.
     */
    bool isDAG() const;
    /**
     * @brief Initializes a single source for the graph.
     *
     * @param origin The origin vertex.
     * @return A pointer to the origin vertex.
     */
    Vertex<T> * initSingleSource(const T &origin);
    /**
     * @brief Relaxes an edge between two vertices.
     *
     * @param v The first vertex.
     * @param w The second vertex.
     * @param weight The weight of the edge.
     * @return true if the relaxation was successful, false otherwise.
     */
    inline bool relax(Vertex<T> *v, Vertex<T> *w, double weight);
    /**
     * @brief Computes the shortest path in an unweighted graph.
     *
     * @param orig The origin vertex.
     */
    void unweightedShortestPath(const T &orig);

    /**
    * @brief Calculates the maximum number of trains that can travel from a source station to a destination station in a weighted graph.
    *
    * The function first performs an unweighted shortest path search from the source station to the destination station, and then, for each vertex
    * in the shortest path, finds the minimum weight of its adjacent edges. The minimum weight is returned as the maximum number of trains that can
    * travel from the source station to the destination station.
    *
    * @param orig The source station.
    * @param dest The destination station.
    * @return The maximum number of trains that can travel from the source station to the destination station, or NULL if there is no path between them.
    */
    int maxNumberTrain(const T &orig, const T &dest);

    /**
     * Returns a vector with the path from the source vertex to the destination vertex.
    * @param dest The destination vertex.
    * @return A vector with the path from the source vertex to the destination vertex.
     * If no path exists, returns an empty vector.
     */
    vector<T> getPath(const T &dest) const;
    /**
     * @brief Computes the shortest path in a weighted graph using Dijkstra's algorithm.
     *
     * @param origin The origin vertex.
     */
    void dijkstraShortestPath(const T &origin);
    /**
     * @brief Computes the shortest path in a weighted graph using Dijkstra's algorithm and returns the path.
     *
     * @param origin The origin vertex.
     * @param dest The destination vertex.
     */
    void dijkstraShortestPath2(const T &origin,const T &dest);
    /**
     * @brief Computes the shortest path in a weighted graph using Dijkstra's algorithm and returns the path distance.
     *
     * @param origin The origin vertex.
     * @param dest The destination vertex.
     * @return The path distance.
     */
    int dijkstraShortestPath3(const T &origin,const T &dest);
    /**
     * @brief Gets a vector containing all vertices in the graph.
     *
     * @return A vector containing all vertices in the graph.
     */
    std::vector<Vertex<T> *> getVertexSet() const;
    std::unordered_map<T, std::unordered_map<T, double>> adjacencyList;
    unordered_map<T, std::vector<Edge<T>>> getEdges() const;

};

template<class T>
unordered_map<T, vector<Edge<T>>> Graph<T>::getEdges() const {
    std::unordered_map<T, std::vector<Edge<T>>> edges;
    for (const auto& [vertex, neighbors] : adjacencyList) {
        std::vector<Edge<T>> edgeList;
        for (const auto& neighbor : neighbors) {
            edgeList.push_back(Edge<T>(vertex, neighbor.first, neighbor.second));
        }
        edges.emplace(vertex, edgeList);
    }
    return edges;
}


template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet) {
        if (v->info == in) {
            return v;
        }
    }
    return NULL;
}


template<class T>
Vertex<T> *Graph<T>::findVertexTag(string tag) const {
    for (auto v : vertexSet)
        if (v->info.getTag() == tag)
            return v;
    return NULL;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if ( findVertex(in) != NULL)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    v1->addEdge(v2,w);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == NULL || v2 == NULL)
        return false;
    return v1->removeEdgeTo(v2);
}

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++)
        if ((*it)->info  == in) {
            auto v = *it;
            vertexSet.erase(it);
            for (auto u : vertexSet)
                u->removeEdgeTo(v);
            delete v;
            return true;
        }
    return false;
}

/*
 * Performs a depth-first search (dfs) in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T>
std::vector<T> Graph<T>::dfs() const {
    std::vector<T> res;
    for (auto v : vertexSet)
        v->visited = false;

    for (auto v : vertexSet)
        if (! v->visited)
            dfsVisit(v, res);
    return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->visited = true;
    res.push_back(v->info);
    for (auto & e : v->adj) {
        auto w = e.dest;
        if ( ! w->visited)
            dfsVisit(w, res);
    }
}


/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T>
std::vector<T> Graph<T>::bfs(const T & source) const {
    std::vector<T> res;
    auto s = findVertex(source);
    if (s == NULL)
        return res;
    queue<Vertex<T> *> q;
    for (auto v : vertexSet)
        v->visited = false;
    q.push(s);
    s->visited = true;
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        res.push_back(v->info);
        for (auto & e : v->adj) {
            auto w = e.dest;
            if ( ! w->visited ) {
                q.push(w);
                w->visited = true;
            }
        }
    }
    return res;
}


/*
 * Performs a topological sorting of the vertices of a graph (this).
 * Returns a vector with the contents of the vertices by topological order.
 * If the graph has cycles, returns an empty vector.
 * Follows the algorithm described in theoretical classes.
 */

template<class T>
std::vector<T> Graph<T>::topsort() const {
    std::vector<T> res;

    for (auto v : vertexSet)
        v->indegree = 0;
    for (auto v : vertexSet)
        for (auto & e : v->adj)
            e.dest->indegree++;

    std::queue<Vertex<T>*> q;
    for (auto v : vertexSet)
        if (v->indegree == 0)
            q.push(v);

    while( !q.empty() ) {
        Vertex<T>* v = q.front();
        q.pop();
        res.push_back(v->info);
        for(auto & e : v->adj) {
            auto w = e.dest;
            w->indegree--;
            if(w->indegree == 0)
                q.push(w);
        }
    }

    if ( res.size() != vertexSet.size() ) {
        //std::cout << "Ordenacao Impossivel!" << std::endl;
        res.clear();
        return res;
    }

    return res;
}


/*
 * Performs a breadth-first search in a graph (this), starting
 * from the vertex with the given source contents (source).
 * During the search, determines the vertex that has a maximum number
 * of new children (adjacent not previously visited), and returns the
 * contents of that vertex and the number of new children.
 */

template <class T>
int Graph<T>::maxNewChildren(const T & source, T &inf) const {
    auto s = findVertex(source);
    if (s == NULL)
        return 0;
    std::queue<Vertex<T> *> q;
    int maxChildren = 0;
    inf = s->info;
    for (auto v : vertexSet)
        v->visited = false;
    q.push(s);
    s->visited = true;
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        int nChildren=0;
        for (auto & e : v->adj) {
            auto w = e.dest;
            if ( ! w->visited ) {
                w->visited = true;
                q.push(w);
                nChildren++;
            }
        }
        if (nChildren>maxChildren) {
            maxChildren = nChildren;
            inf = v->info;
        }
    }
    return maxChildren;
}


/*
 * Performs a depth-first search in a graph (this), to determine if the graph
 * is acyclic (acyclic directed graph or DAG).
 * During the search, a cycle is found if an edge connects to a vertex
 * that is being processed in the the stack of recursive calls (see theoretical classes).
 * Returns true if the graph is acyclic, and false otherwise.
 */

template <class T>
bool Graph<T>::isDAG() const {
    for (auto v : vertexSet) {
        v->visited = false;
        v->processing = false;
    }
    for (auto v : vertexSet)
        if (! v->visited)
            if ( ! dfsIsDAG(v) )
                return false;
    return true;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Returns false (not acyclic) if an edge to a vertex in the stack is found.
 */
template <class T>
bool Graph<T>::dfsIsDAG(Vertex<T> *v) const {
    v->visited = true;
    v->processing = true;
    for (auto & e : v->adj) {
        auto w = e.dest;
        if (w->processing)
            return false;
        if (! w->visited)
            if (! dfsIsDAG(w))
                return false;
    }
    v->processing = false;
    return true;
}



/*
* Analyzes an edge in single source shortest path algorithm.
* Returns true if the target vertex was relaxed (dist, path).
* Used by all single-source shortest path algorithms.
*/
template<class T>
inline bool Graph<T>::relax(Vertex<T> *v, Vertex<T> *w, double weight) {
    if (v->dist + weight < w->dist) {
        w->dist = v->dist + weight;
        w->path = v;
        return true;
    }
    else
        return false;
}

template<class T>
void Graph<T>::unweightedShortestPath(const T &orig) {
    auto s = initSingleSource(orig);
    queue< Vertex<T>* > q;
    q.push(s);
    while( ! q.empty() ) {
        auto v = q.front();
        q.pop();
        for(auto e: v->adj)
            if (relax(v, e.dest, 1))
                q.push(e.dest);
    }
}

template<class T>
int Graph<T>::maxNumberTrain(const T &orig, const T &dest) {
    unweightedShortestPath(orig);
    vector<Station*> v = getPath(dest);
    if (v.size() == 0) {
        return NULL;
    }
    auto s = initSingleSource(orig);
    queue< Vertex<T>* > q;
    q.push(s);
    int w = 255;
    while( ! q.empty() ) {
        auto v = q.front();
        q.pop();
        for (auto e: v->adj) {
            if (e.weight < w){
                w = e.weight;
            }
            if (e.dest == findVertex(dest))
                break;
            if (relax(v, e.dest, 1))
                q.push(e.dest);
        }
    }
    if (w == 255)
        return NULL;
    return w;
}





template<class T>
vector<T> Graph<T>::getPath(const T &dest) const{
    vector<T> res;
    auto v = findVertex(dest);
    if (v == nullptr || v->dist == INF) // missing or disconnected
        return res;
    for ( ; v != nullptr; v = v->path)
        res.push_back(v->info);
    reverse(res.begin(), res.end());
    return res;
}

template<class T>
void Graph<T>::dijkstraShortestPath(const T &origin) {
    auto s = initSingleSource(origin);
    MutablePriorityQueue<Vertex<T>> q;
    q.insert(s);
    while( ! q.empty() ) {
        auto v = q.extractMin();
        for (auto e: v->adj) {
            auto oldDist = e.dest->dist;
            if (relax(v, e.dest, e.weight)) {
                if (oldDist == INF)
                    q.insert(e.dest);
                else
                    q.decreaseKey(e.dest);
            }
        }
    }

}

template<class T>
void Graph<T>::dijkstraShortestPath2(const T &origin,const T &dest) {
    this->dijkstraShortestPath(origin);

    Vertex<T>* temp = this->findVertex(dest);
    std::vector<T> path;
    bool noPath = false;
    while(temp->info.getStationName() != origin.getStationName()) {
        path.push_back(temp->info);
        temp = temp->path;
        if(temp == NULL){
            noPath = true;
            break;
        }
    }
    if(noPath) {
        std::cout << "No path found!" << std::endl;
        return;
    }
    else {
        for(Station s : path) {
            std::cout << s.getCode() << std::endl;
        }
        std::cout << origin.getStationName() << std::endl;
    }

}

template<class T>
int Graph<T>::dijkstraShortestPath3(const T &origin,const T &dest) {
    auto s = initSingleSource(origin);
    MutablePriorityQueue<Vertex<T>> q;
    q.insert(s);
    int w = 255;
    while( ! q.empty() ) {
        auto v = q.extractMin();
        for (auto e: v->adj) {
            auto oldDist = e.dest->dist;
            if (e.weight < w){
                w = e.weight;
            }
            if (e.dest == findVertex(dest))
                break;
            if (relax(v, e.dest, e.weight)) {
                if (oldDist == INF)
                    q.insert(e.dest);
                else
                    q.decreaseKey(e.dest);
            }
        }
    }
    return w;
}

/*
* Initializes single source shortest path data (path, dist).
* Receives the content of the source vertex and returns a pointer to
the source vertex.
* Used by all single-source shortest path algorithms.
*/
template<class T>
Vertex<T> * Graph<T>::initSingleSource(const T &origin) {
    for(auto v : vertexSet) {
        v->dist = INF;
        v->path = nullptr;
    }
    auto s = findVertex(origin);
    s->dist = 0;
    return s;
}


template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}



/**************** Single Source Shortest Path algorithms ************/

template <class T>
void deleteMatrix(T **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}
#endif //CAL_PROJ_GRAPH_H