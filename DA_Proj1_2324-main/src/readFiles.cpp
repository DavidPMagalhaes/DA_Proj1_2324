#include "ReadFiles.h"
#include <fstream>
#include <sstream>
#include <algorithm>

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

void ReadFiles::readReservoirsData(const std::string& filename, Graph& graph) {
    // Implementation of readReservoirsData
}

void ReadFiles::readStationsData(const std::string& filename, Graph& graph) {
    // Implementation of readStationsData
}
