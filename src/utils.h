#include "readFiles.h"
#include <unordered_set>
#include <set>

/**
 * @brief Solves the traveling salesman problem using the backtracking algorithm.
 *
 * This function finds the shortest tour that visits all the stations in the network
 * using the backtracking algorithm. It starts from a given source node and explores
 * all possible paths, backtracking when necessary, until all nodes are visited.
 *
 * @param graph The graph representing the network of stations and connections.
 * @param src The index of the source node to start the tour.
 * @param tour A vector to store the indices of the nodes in the tour.
 * @param stations A vector of all the stations in the network.
 * @param networks A vector of all the network connections between stations.
 *
 * @return The total distance of the shortest tour.
 */
float tspBacktrackingAlgorithm(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks);

/**
 * @brief Calculate the total distance of a given tour in the graph.
 *
 * This function calculates the total distance of a given tour in the graph by summing up the weights (distances)
 * of the network connections between consecutive stations in the tour.
 *
 * @param graph The graph representing the network of stations and connections.
 * @param tour The vector representing the tour of stations.
 * @param stations The vector of stations.
 * @param networks The vector of network connections.
 * @return The total distance of the tour.
 */
float calculateTotalDistance(Graph<Station*> *graph, const vector<int>& tour, vector<Station*> stations, vector<Network*> networks);

/**
 * @brief Triangular Approximation Heuristic for the Traveling Salesman Problem.
 *
 * This function applies the Triangular Approximation Heuristic to approximate the solution for the
 * Traveling Salesman Problem (TSP) in a given graph. The TSP aims to find the shortest tour that visits
 * all nodes (stations) exactly once and returns to the starting node.
 *
 * The algorithm starts from a given source node (src) and iteratively selects the next node to visit based on
 * the nearest unvisited neighbor. It constructs a tour by greedily connecting nodes with the shortest network
 * connections (edges). The process continues until all nodes are visited or no unvisited neighbors are found.
 *
 * @param graph The graph representing the network of stations and connections.
 * @param src The source node (starting station) for the TSP.
 * @param tour The vector to store the resulting tour of stations.
 * @param stations The vector of stations in the graph.
 * @param networks The vector of network connections in the graph.
 * @return The total distance of the resulting tour.
 */
float tspApproximationAlgorithm(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks);

/**
 * @brief Solves the traveling salesman problem using the nearest neighbor algorithm.
 *
 * This function finds the shortest tour that visits all the stations in the network
 * using the nearest neighbor algorithm. It starts from a given source node and at each step,
 * it chooses the nearest unvisited neighbor as the next node to visit.
 *
 * @param graph The graph representing the network of stations and connections.
 * @param src The index of the source node to start the tour.
 * @param tour A vector to store the indices of the nodes in the tour.
 * @param stations A vector of all the stations in the network.
 * @param networks A vector of all the network connections between stations.
 *
 * @return The total distance of the shortest tour.
 */
float tspNearestNeighbor(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks);