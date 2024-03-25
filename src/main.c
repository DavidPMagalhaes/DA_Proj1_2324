
#include <iostream>
#include <vector>
#include "istream"
#include "readFiles.h"
#include "utils.cpp"


using namespace std;

void option1(vector<Station *> stations);
void option2(vector<Station *> stations);
void option3(vector<Station *> stations);
void option4(vector<Station *> stations);
void option5(vector<Station *> stations, vector <Network*> networks);
void option6(vector<Station *> stations);

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

/**
 * @brief Select option 1
 *
 * Allows the user to view the maximum number of trains that can simultaneously travel between two specific stations.
 * Asks for starting and destination stations and prints the result to the console.
 *
 * @param stations Vector of pointers to Station objects
 */
void option1(vector<Station *> stations) {
    cout << "Selected first option" << endl;
    cout << "View max number of trains that can simultaneously travel between two specific stations" << endl;
    cout << "Input starting station" << endl;
    cout << "->";
    string start;
    cin.ignore();
    getline(std::cin, start);
    cout << "Input destination station: " << endl;
    string end;
    getline(std::cin, end);
    cout << "Starting station: " << start << std::endl;
    cout << "Destination station: " << end << std::endl;
    Station *src = findStation(start, stations);
    Station *dest = findStation(end, stations);
    maxNumberTrains(&g, src, dest);
}

/**
 * @brief Select option 2
 *
 * Allows the user to view the shortest path between two stations.
 * Asks for starting and destination stations and prints the result to the console.
 *
 * @param stations Vector of pointers to Station objects
 */
void option2(vector<Station *> stations) {
    cout << "Selected second option" << endl;
    cout << "View shortest path between two stations" << endl;
    cout << "Input starting station" << endl;
    cout << "->";
    string start;
    cin.ignore();
    getline(std::cin, start);
    cout << "Input destination station: " << endl;
    string end;
    getline(std::cin, end);
    cout << "Starting station: " << start << std::endl;
    cout << "Destination station: " << end << std::endl;
    Station *src = findStation(start, stations);
    Station *dest = findStation(end, stations);
    shortestPath(&g, src, dest);
}

/**
 * @brief Select option 3
 *
 * Allows the user to determine which pairs of stations require the most amount of trains.
 * Prints the result to the console.
 *
 * @param stations Vector of pointers to Station objects
 */
void option3(vector<Station *> stations) {
    cout << "Selected fourth option" << endl;
    cout << "Determine which pairs of stations require most amount of trains" << endl;
    mostAmount(&g, stations);
}

/**
 * @brief Select option 4
 *
 * Allows the user to report the max number of trains that can simultaneously arrive at a given station
 * Asks for destination station and prints the result to the console.
 *
 * @param stations Vector of pointers to Station objects
 */
void option4(vector<Station *> stations) {
    cout << "Selected fourth option" << endl;
    cout << "Report the maximum number of trains that can simultaneously arrive at a given station,\n"
            "taking into consideration the entire railway grid." << endl;
    cout << "Input destination station: " << endl;
    string end;
    cin.ignore();
    getline(std::cin, end);
    Station *dest = findStation(end, stations);
    maxNumberTrainsAllStations(&g, dest, stations);
}

/**
 * @brief Select option 5
 *
 * Allows the user to alculate the max number of trains that can simultaneously travel
 * between two stations at minimum cost to the company.
 *
 * @param stations Vector of pointers to Station objects
 * @param networks Vector of pointers to Network objects
 */
void option5(vector<Station *> stations, vector <Network*> networks) {
    cout << "Selected sixth option" << endl;
    cout << "Calculate the max amount of trains that can simultaneously travel between two specific "
            "stations with minimum cost for the company. Note that your system should also take any valid "
            "source and destination stations as input;" << endl;
    cout << "Input starting station" << endl;
    cout << "->";
    string start;
    cin.ignore();
    getline(std::cin, start);
    cout << "Input destination station: " << endl;
    string end;
    getline(std::cin, end);
    cout << "Starting station: " << start << std::endl;
    cout << "Destination station: " << end << std::endl;
    Station *src = findStation(start, stations);
    Station *dest = findStation(end, stations);
    calculatePrice(src, dest, &g, &networks);
}
/**
 * @brief Select option 6
 *
 * Allows the user to calculate the max number of trains that can simultaneously travel
 * between two stations in a network of reduced connectivity.
 *
 * @param stations Vector of pointers to Station objects
 */
void option6(vector<Station *> stations) {
    cout << "Selected sixth option" << endl;
    cout << "Calculate the max number of trains that can simultaneously travel between two stations at reduced connectivity" << endl;
    cout << "Input starting station" << endl;
    cout << "->";
    string start;
    cin.ignore();
    getline(std::cin, start);
    cout << "Input destination station: " << endl;
    string end;
    getline(std::cin, end);
    cout << "Starting station: " << start << std::endl;
    cout << "Destination station: " << end << std::endl;
    Station *src = findStation(start, stations);
    Station *dest = findStation(end, stations);

    Graph<Station*> soup = subgraph(stations, src, dest);
    cout << "Subgraph created successfully" << endl;
    maxNumberTrains(&soup, src, dest);

}