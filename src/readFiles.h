#pragma once
#include "Network.h"
#include "fstream"
#include "Graph/Graph.h"
#include <iostream>
#include <cstring>
#include <utility>
#include <string>
#include <vector>
#include <sstream>

extern Graph<Station*> g;
/**
 * @brief Read network connections from a file and create Network objects.
 *
 * This function reads network connections from a CSV file and creates Network objects based on the data.
 * It takes the selected filename and a vector of Station pointers as input.
 *
 * @param selected The selected filename without the file extension.
 * @param nodes A vector of Station pointers to search for source and destination stations.
 * @return A vector of Network pointers containing the created network connections.
 */
vector<Network*> readNetworks(string selected, vector<Station*> nodes);

/**
 * @brief Read the nodes from a CSV file.
 *
 * Reads the nodes from a CSV file and returns them as a vector of Station pointers.
 *
 * @param selected A string representing the selected value.
 * @return A vector of Station pointers containing the read nodes.
 */
vector<Station*> readNodes(string selected);

/**
 * @brief Find a node in a vector of Station pointers.
 *
 * This function searches for a node with the specified code in a vector of Station pointers.
 *
 * @param code The code of the node to search for.
 * @param nodes A vector of Station pointers to search within.
 * @return A pointer to the found Station, or NULL if no match is found.
 */
Station* findNode(int code, vector<Station*> nodes);


/**
 * @brief Finds a network that has a station with a given name
 *
 * @param name The name of the station to search for
 * @param networks A vector containing all the networks to search through
 * @return A pointer to the network that contains a station with the given name, or NULL if not found
 */
Network* findNetwork(Station* name, vector<Network*> network);

/**
 * @brief Find a network connection between two specific stations.
 *
 * This function searches for a network connection in a vector of Network pointers between two specified
 * stations
 * @param name A pointer to the first station.
 * @param name2 A pointer to the second station.
 * @param networks A vector of Network pointers to search within.
 * @return A pointer to the Network representing the network connection between the two specified stations,
 * or NULL if no connection is found.
 */
Network* findNetwork2(Station* name, Station* name2, vector<Network*> networks);

/**
 * Splits a string into a vector of substrings based on a given delimiter character.
 *
 * @param str The input string that needs to be split into substrings based on the delimiter.
 * @param del The delimiter character used to split the input string into smaller substrings.
 *
 * @return a vector of strings that contains the substrings of the input string that were separated by the specified
 * delimiter character.
 */
vector<string> split(const string &str, const char del);

/**
 * @brief Get the weight of the network connection between two stations.
 *
 * This function searches for a network connection between two stations in a vector of Network pointers
 * and returns the weight of that connection.
 *
 * @param dest A pointer to the destination station.
 * @param src A pointer to the source station.
 * @param networks A vector of Network pointers to search within.
 * @return The weight of the network connection between the source and destination stations, or NULL if no
 * connection is found.
 */
int getWeight(Station* dest, Station *src, vector<Network*> networks);

/**
 * @brief Find neighbors of a specific station in a vector of network connections.
 *
 * This function searches for network connections in a vector of Network pointers that are connected
 * to a specific station.
 *
 * @param name A pointer to the station for which to find neighbors.
 * @param networks A vector of Network pointers to search within.
 * @return A vector of Network pointers representing the neighbors of the specified station.
 */
vector<Network*> findNeighbors(Station* name, vector<Network*> networks);

#define DA2223_2_READFILES_H


