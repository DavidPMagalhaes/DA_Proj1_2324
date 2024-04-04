// ReadFiles.h
#ifndef READFILES_H
#define READFILES_H

#include <string>
#include <fstream>
#include <sstream>
#include "Graph.h"

class ReadFiles {
public:
    static void readCitiesData(const std::string& filename, Graph& graph);
    static void readPipesData(const std::string& filename, Graph& graph);
    static void readReservoirsData(const std::string& filename, Graph& graph);
    static void readStationsData(const std::string& filename, Graph& graph);
};

#endif // READFILES_H
