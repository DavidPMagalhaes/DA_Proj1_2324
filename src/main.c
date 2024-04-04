
#include <iostream>
#include <vector>
#include "istream"
#include "readFiles.h"
#include "utils.cpp"


using namespace std;

/**
 * @brief Compute the maximum flow from reservoirs to cities
 *
 * @param graph The water supply network graph
 * @param reservoirs The reservoirs in the network
 * @param cities The cities (delivery sites) in the network
 * @return vector<pair<string, double>> Maximum flow to each city
 */
vector<pair<string, double>> computeMaxFlow(Graph& graph, const vector<string>& reservoirs, const vector<string>& cities) {
vector<pair<string, double>> maxFlows;

// Create a Ford-Fulkerson object for computing maximum flow
FordFulkerson fordFulkerson(graph);

// Iterate over each city
for (const string& city : cities) {
double maxFlow = 0.0;

// Find the vertex corresponding to the city
Vertex* cityVertex = graph.getVertex(city);

// Iterate over each reservoir
for (const string& reservoir : reservoirs) {
// Find the vertex corresponding to the reservoir
Vertex* reservoirVertex = graph.getVertex(reservoir);

// Compute maximum flow from the reservoir to the city using Ford-Fulkerson algorithm
double flow = fordFulkerson.getMaxFlow(reservoirVertex, cityVertex);

// Update maxFlow if the flow from the current reservoir is greater
maxFlow = max(maxFlow, flow);
}

// Add the pair (city, maxFlow) to the result vector
maxFlows.emplace_back(city, maxFlow);
}

return maxFlows;
}

/**
 * @brief Option 1: Determine the maximum amount of water that can reach each or a specific city
 *
 * This function reads the data from CSV files, forms the network graph, and computes the maximum flow
 * to each city using the Ford-Fulkerson algorithm.
 */
void option1() {
    // Read data from CSV files
    ReadFiles readFiles;
    readFiles.readReservoirs("Reservoirs_Madeira.csv");
    readFiles.readStations("Stations_Madeira.csv");
    readFiles.readCities("Cities_Madeira.csv");
    readFiles.readPipes("Pipes_Madeira.csv");

    // Get data from ReadFiles object
    vector<string> reservoirs = readFiles.getReservoirs();
    vector<string> stations = readFiles.getStations();
    vector<string> cities = readFiles.getCities();
    vector<vector<string>> pipesData = readFiles.getPipesData();

    // Create a graph object
    Graph graph;

    // Form the network from parsed input data
    graph.formNetwork(reservoirs, stations, cities, pipesData);

    // Compute maximum flow to each city
    vector<pair<string, double>> maxFlows = computeMaxFlow(graph, reservoirs, cities);

    // Print the results
    cout << "Maximum flow to each city:" << endl;
    for (const auto& [city, maxFlow] : maxFlows) {
        cout << city << ": " << maxFlow << " m3/sec" << endl;
    }
}


/**
 * @brief Option 2: View shortest path between two stations
 *
 * This function reads the data from CSV files, forms the network graph, and finds the shortest path
 * between two specified stations using Dijkstra's algorithm.
 */
void option2() {
    // Read data from CSV files
    ReadFiles readFiles;
    readFiles.readReservoirs("Reservoirs_Madeira.csv");
    readFiles.readStations("Stations_Madeira.csv");
    readFiles.readCities("Cities_Madeira.csv");
    readFiles.readPipes("Pipes_Madeira.csv");

    // Get data from ReadFiles object
    vector<string> reservoirs = readFiles.getReservoirs();
    vector<string> stations = readFiles.getStations();
    vector<string> cities = readFiles.getCities();
    vector<vector<string>> pipesData = readFiles.getPipesData();

    // Create a graph object
    Graph graph;

    // Form the network from parsed input data
    graph.formNetwork(reservoirs, stations, cities, pipesData);

    // Create a Dijkstra object for finding shortest paths
    Dijkstra dijkstra(graph);

    // Get source and destination stations from the user
    string source, destination;
    cout << "Enter source station code: ";
    cin >> source;
    cout << "Enter destination station code: ";
    cin >> destination;

    // Find the shortest path between the source and destination stations
    vector<Vertex*> shortestPath = dijkstra.getShortestPath(graph.getVertex(source), graph.getVertex(destination));

    // Print the shortest path
    if (!shortestPath.empty()) {
        cout << "Shortest path from " << source << " to " << destination << ":" << endl;
        for (Vertex* vertex : shortestPath) {
            cout << vertex->getId() << " -> ";
        }
        cout << endl;
    } else {
        cout << "No path found between " << source << " and " << destination << endl;
    }
}

#include <iostream>
#include <vector>
#include "readFiles.h"
#include "Graph.h"
#include "Vertex.h"
#include "Edge.h"

using namespace std;

/**
 * @brief Option 3: Check full water supply to delivery sites (cities)
 *
 * This function reads the data from CSV files, forms the network graph, and checks if each delivery site
 * (city) in the network receives water from at least one reservoir or pumping station.
 */
void option3() {
    // Read data from CSV files
    ReadFiles readFiles;
    readFiles.readReservoirs("Reservoirs_Madeira.csv");
    readFiles.readStations("Stations_Madeira.csv");
    readFiles.readCities("Cities_Madeira.csv");
    readFiles.readPipes("Pipes_Madeira.csv");

    // Get data from ReadFiles object
    vector<string> reservoirs = readFiles.getReservoirs();
    vector<string> stations = readFiles.getStations();
    vector<string> cities = readFiles.getCities();
    vector<vector<string>> pipesData = readFiles.getPipesData();

    // Create a graph object
    Graph graph;

    // Form the network from parsed input data
    graph.formNetwork(reservoirs, stations, cities, pipesData);

    bool allSupplied = true;

    // Check water supply to each city
    for (const string& city : cities) {
        // Find the vertex corresponding to the city
        Vertex* cityVertex = graph.getVertex(city);

        // Check if there is a path from any reservoir or pumping station to the city
        bool supplied = false;
        for (const string& reservoir : reservoirs) {
            // Find the vertex corresponding to the reservoir
            Vertex* reservoirVertex = graph.getVertex(reservoir);
            // Check if there is a path from reservoir to city
            if (graph.isReachable(reservoirVertex, cityVertex)) {
                supplied = true;
                break;
            }
        }
        if (!supplied) {
            cout << "City " << city << " does not receive water supply." << endl;
            allSupplied = false;
        }
    }

    if (allSupplied) {
        cout << "All cities receive water supply." << endl;
    }
}

/**
 * @brief Option 4: Minimize the differences of flow to capacity on each pipe across the entire network
 *
 * This function reads the data from CSV files, forms the network graph, computes the flow through each pipe
 * using the Ford-Fulkerson algorithm, and adjusts the flow to minimize differences with capacities.
 */
void option4() {
    // Read data from CSV files
    ReadFiles readFiles;
    readFiles.readReservoirs("Reservoirs_Madeira.csv");
    readFiles.readStations("Stations_Madeira.csv");
    readFiles.readCities("Cities_Madeira.csv");
    readFiles.readPipes("Pipes_Madeira.csv");

    // Get data from ReadFiles object
    vector<string> reservoirs = readFiles.getReservoirs();
    vector<string> stations = readFiles.getStations();
    vector<string> cities = readFiles.getCities();
    vector<vector<string>> pipesData = readFiles.getPipesData();

    // Create a graph object
    Graph graph;

    // Form the network from parsed input data
    graph.formNetwork(reservoirs, stations, cities, pipesData);

    // Create a Ford-Fulkerson object for computing maximum flow
    FordFulkerson fordFulkerson(graph);

    // Iterate over each edge (pipe) in the graph
    for (Edge* edge : graph.getEdges()) {
        // Get the capacity of the current edge
        double capacity = edge->getCapacity();

        // Compute the maximum flow through the current edge using Ford-Fulkerson algorithm
        double flow = fordFulkerson.getMaxFlow(edge->getSource(), edge->getDestination());

        // Adjust the flow to minimize differences with capacity
        double adjustedFlow = min(flow, capacity); // Adjusted flow cannot exceed capacity

        // Update the flow on the current edge
        edge->setFlow(adjustedFlow);
    }

    // Print the adjusted flows on each pipe
    cout << "Adjusted flows on each pipe:" << endl;
    for (Edge* edge : graph.getEdges()) {
        cout << "From " << edge->getSource()->getId() << " to " << edge->getDestination()->getId()
             << ": Flow = " << edge->getFlow() << ", Capacity = " << edge->getCapacity() << endl;
    }
}

/**
 * Displays the main menu on the console
 */
void menu() {
    cout << "|-------------------------------------------------------------------------------------------------------------|" << endl;
    cout << "| __          __   _               _____                   _         __  __                                   |" << endl;
    cout << "| \\ \\        / /  | |             / ____|                 | |       |  \\/  |                                  |" << endl;
    cout << "|  \\ \\  /\\  / /_ _| |_ ___ _ __  | (___  _   _ _ __  _ __ | |_   _  | \\  / | __ _ _ __   __ _  __ _  ___ _ __ ||" << endl;
    cout << "|   \\ \\/  \\/ / _` | __/ _ \\ '__|  \\___ \\| | | | '_ \\| '_ \\| | | | | | |\\/| |/ _` | '_ \\ / _` |/ _` |/ _ \\ '__|" << endl;
    cout << "|    \\  /\\  / (_| | ||  __/ |     ____) | |_| | |_) | |_) | | |_| | | |  | | (_| | | | | (_| | (_| |  __/ |   |" << endl;
    cout << "|     \\/  \\/ \\__,_|\\__\\___|_|    |_____/ \\__,_| .__/| .__/|_|\\__, | |_|  |_|\\__,_|_| |_|\\__,_|\\__, |\\___|_|   |" << endl;
    cout << "|                                             | |   | |       __/ |                            __/ |          |" << endl;
    cout << "|                                             |_|   |_|      |___/                            |___/           |" << endl;
    cout << "|-------------------------------------------------------------------------------------------------------------|" << endl;
    cout << "(1) Determine the maximum amount of water that can reach each or a specific city (T2.1)" << endl;
    cout << "(2) View shortest path between two stations" << endl;
    cout << "(3) Check full water supply to delivery sites (T2.2)" << endl;
    cout << "(4) Minimize the differences of flow to capacity on each pipe across the entire network (T2.3)" << endl;
    cout << "(0) Exit" << endl;
    cout << "->";
}

/**
 * @brief Ask user whether to continue or exit the program
 *
 * Prints a prompt to the console asking the user whether to continue
 * running the program or exit. If user enters 0, the function will
 * print a message indicating that the program is exiting and exit the program.
 */
void check() {
    cout << "Continue?";
    cout << "(1) Yes" << endl;
    cout << "(0) No" << endl;
    int input;
    cin >> input;
    if (input==0) {
        std::cout << "Exiting program..." << std::endl;
        exit(0);
    }
}

/**
 * @brief Main function of the program
 * The main function of the program. Creates two graphs - station and network, reads
 * all stations and networks, presents a menu of options for the user to choose from.
 */
int main(int argc, char const *argv[])
{
    //Create two graphs - station and network
    //Read all stations
    //Read all networks
    bool loop = true;

    //vector <Station*> stations = readStations();
    //vector <Network*> networks = readNetworks(stations);
    while (loop)
    {
        menu();
        int input_menu_principal;
        std::cin >> input_menu_principal;
        switch (input_menu_principal)
        {
            case 1:
            {
                //option1(stations);
                check();
                break;
            }
            case 2:
            {
                //option2(stations);
                check();
                break;
            }
            case 3:
            {
                //option3(stations);
                check();
                break;
            }
            case 4: {
                //option4(stations);
                check();
                break;
            }
            case 0: {
                std::cout << "Exiting program..." << std::endl;
                return 0;
            }
            default:
                std::cout << "Number not found" << std::endl;
        }
    }
}