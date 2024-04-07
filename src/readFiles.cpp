/**
 * @file ReadFiles.cpp
 * @brief Implementation of the ReadFiles class
 */

#include "ReadFiles.h"
#include <fstream>
#include <sstream>
#include <algorithm>

/**
 * @brief Reads cities data from a file and populates the graph.
 * @param filename Name of the file containing cities data
 * @param graph Reference to the graph to populate
 */
void ReadFiles::readCitiesData(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Read and discard header line
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string city, id, code, demandStr, populationStr;
        double demand;
        int population;
        if (std::getline(iss, city, ',') && std::getline(iss, id, ',') &&
            std::getline(iss, code, ',') && std::getline(iss, demandStr, ',') &&
            std::getline(iss, populationStr, ',')) {
            // Remove quotes from demand and population strings
            demandStr.erase(std::remove_if(demandStr.begin(), demandStr.end(), [](char c) { return c == '"'; }), demandStr.end());
            populationStr.erase(std::remove_if(populationStr.begin(), populationStr.end(), [](char c) { return c == '"'; }), populationStr.end());
            // Convert demand and population strings to double and int respectively
            demand = std::stod(demandStr);
            population = std::stoi(populationStr);
            // Create a vertex for the city and add it to the graph
            Vertex vertex(code, VertexType::DELIVERY_SITE); // Assuming cities are delivery sites
            graph.addVertex(vertex);
        }
    }
    file.close();
}

/**
 * @brief Reads pipes data from a file and populates the graph.
 * @param filename Name of the file containing pipes data
 * @param graph Reference to the graph to populate
 */
void ReadFiles::readPipesData(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Read and discard header line
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string servicePointA, servicePointB, capacityStr, directionStr;
        double capacity;
        int direction;
        if (std::getline(iss, servicePointA, ',') && std::getline(iss, servicePointB, ',') &&
            std::getline(iss, capacityStr, ',') && std::getline(iss, directionStr, ',')) {
            // Convert capacity and direction strings to double and int respectively
            capacity = std::stod(capacityStr);
            direction = std::stoi(directionStr);
            // Assuming connections between service points are pipelines
            Edge edge(&servicePointA, &servicePointB, capacity, true, EdgeType::PIPELINE);
            graph.addEdge(edge);
        }
    }
    file.close();
}

/**
 * @brief Reads reservoirs data from a file and populates the graph.
 * @param filename Name of the file containing reservoirs data
 * @param graph Reference to the graph to populate
 */
void ReadFiles::readReservoirsData(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Read and discard header line
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string reservoir, municipality, id, code, maxDeliveryStr;
        double maxDelivery;
        if (std::getline(iss, reservoir, ',') && std::getline(iss, municipality, ',') &&
            std::getline(iss, id, ',') && std::getline(iss, code, ',') &&
            std::getline(iss, maxDeliveryStr, ',')) {
            // Convert maxDelivery string to double
            maxDelivery = std::stod(maxDeliveryStr);
            // Create a vertex for the reservoir and add it to the graph
            Vertex vertex(code, VertexType::RESERVOIR);
            graph.addVertex(vertex);
        }
    }
    file.close();
}

/**
 * @brief Reads stations data from a file and populates the graph.
 * @param filename Name of the file containing stations data
 * @param graph Reference to the graph to populate
 */
void ReadFiles::readStationsData(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    std::string line;
    std::getline(file, line); // Read and discard header line
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string idStr, code;
        int id;
        if (std::getline(iss, idStr, ',') && std::getline(iss, code, ',')) {
            // Convert id string to int
            id = std::stoi(idStr);
            // Create a vertex for the station and add it to the graph
            Vertex vertex(code);
            graph.addVertex(vertex);
        }
    }
    file.close();
}
