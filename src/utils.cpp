//
// Created by bianca on 21-05-2023.
//

#include "utils.h"

float tspBacktrackingAlgorithm(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks)
{
    set<int> visited;
    visited.insert(src);
    tour.push_back(src);
    int current = src;
    while (visited.size() < graph->getNumVertex()-1) // Compare against the number of nodes in the graph
    {
        bool foundNextNode = false;
        for (const auto& neighbor : findNeighbors(findNode(current, stations), networks))
        {
            int neighborNode = neighbor->getCodeB()->getCode();
            if (visited.count(neighborNode) > 0)
                continue;
            tour.push_back(neighborNode);
            visited.insert(neighborNode);
            current = neighborNode;
            foundNextNode = true;
            break;
        }
        if (!foundNextNode)
        {
            int prevNode = tour.back();
            tour.pop_back();
            current = tour.back();
        }
    }
    tour.push_back(src);
    float totalDistance = calculateTotalDistance(graph, tour, stations, networks);
    return totalDistance;
}


float calculateTotalDistance(Graph<Station*> *graph, const vector<int>& tour, vector<Station*> stations, vector<Network*> networks)
{
    float totalDistance = 0.0;
    for (size_t i = 0; i < tour.size() - 1; ++i)
    {
        int currentNode = tour[i];
        int nextNode = tour[i + 1];
        Station* currStation = findNode(currentNode, stations);
        Station* nextStation = findNode(nextNode, stations);
        Network* network = findNetwork2(currStation, nextStation, networks);
        if (network != NULL)
            totalDistance += network->getWeight();
    }

    return totalDistance;
}

float tspApproximationAlgorithm(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks)
{
    set<int> visited;
    visited.insert(src);
    tour.push_back(src);
    int current = src;
    while (visited.size() < stations.size())
    {
        Station* currStation = findNode(current, stations);
        float minDistance = numeric_limits<float>::max();
        int nextNode = -1;
        for (const auto& neighbor : findNeighbors(currStation, networks))
        {
            int neighborNode = neighbor->getCodeB()->getCode();
            if (visited.count(neighborNode) > 0)
                continue;
            float distance = neighbor->getWeight();
            if (distance < minDistance)
            {
                minDistance = distance;
                nextNode = neighborNode;
            }
        }
        if (nextNode == -1)
            break;
        tour.push_back(nextNode);
        visited.insert(nextNode);
        current = nextNode;
    }
    tour.push_back(src);
    float totalDistance = calculateTotalDistance(graph, tour, stations, networks);
    return totalDistance;
}

float tspNearestNeighbor(Graph<Station*> *graph, int src, vector<int>& tour, vector<Station*> stations, vector<Network*> networks)
{
    set<int> visited;
    visited.insert(src);
    tour.push_back(src);
    int current = src;
    while (visited.size() < stations.size())
    {
        Station* currStation = findNode(current, stations);
        float minDistance = numeric_limits<float>::max();
        int nextNode = -1;
        for (const auto& neighbor : findNeighbors(currStation, networks))
        {
            int neighborNode = neighbor->getCodeB()->getCode();
            if (visited.count(neighborNode) > 0)
                continue;
            float distance = neighbor->getWeight();
            if (distance < minDistance)
            {
                minDistance = distance;
                nextNode = neighborNode;
            }
        }
        if (nextNode == -1)
        {
            for (const auto& station : stations)
            {
                if (visited.count(station->getCode()) == 0)
                {
                    nextNode = station->getCode();
                    break;
                }
            }
        }
        tour.push_back(nextNode);
        visited.insert(nextNode);
        current = nextNode;
    }
    tour.push_back(src);
    float totalDistance = calculateTotalDistance(graph, tour, stations, networks);
    return totalDistance;
}

