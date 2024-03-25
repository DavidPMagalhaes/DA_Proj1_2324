#include "readFiles.h"

using namespace std;

Graph<Station*> g;

vector<Station*> readNodes(string selected)
{
    string fname = selected + "nodes.csv";
    vector<vector<string>> content;
    vector<string> row;
    string line, word;

    vector<Station *> nodes;

    fstream file (fname, ios::in);
    if(file.is_open())
    {
        int i = 0;
        getline(file, line);
        while(getline(file, line))
        {
            row.clear();

            stringstream str(line);
            while(getline(str, word, ',')) {
                row.push_back(word);
            }
            Station* stat = new Station(i, stof(row[1]), stof(row[2]));
            // no final do codigo pra cada station fazer "delete station"
            g.addVertex(stat);
            nodes.push_back(stat);
            i++;
        }
    }
    else
        cout<<"Could not open the file\n";

    return nodes;
}

Station* findNode(int code, vector<Station*> nodes){
    for (int i = 0; i < nodes.size(); i++) {
        Station st = *nodes[i];
        int s = nodes[i]->getCode();
        if (s == code)
            return nodes[i];
    }
    return NULL;
}

vector<Network*> findNeighbors(Station* name, vector<Network*> networks){
    vector<Network*> neighbors;
    for (int i = 0; i < networks.size(); i++) {
        Network net = *networks[i];
        int s = networks[i]->getCodeA()->getCode();
        if (s == name->getCode())
            neighbors.push_back(networks[i]);
    }
    return neighbors;
}

int getWeight(Station* dest, Station *src, vector<Network*> networks){
    for (int i = 0; i < networks.size(); i++) {
        Network net = *networks[i];
        int s = networks[i]->getCodeA()->getCode();
        int s2 = networks[i]->getCodeB()->getCode();
        if (s == src->getCode() && s2 == dest->getCode())
            return networks[i]->getWeight();
    }
    return NULL;
}


Network* findNetwork(Station* name, vector<Network*> networks){
    for (int i = 0; i < networks.size(); i++) {
        Network net = *networks[i];
        int s = networks[i]->getCodeA()->getCode();
        if (s == name->getCode())
            return networks[i];
    }
    return NULL;
}

Network* findNetwork2(Station* name, Station* name2, vector<Network*> networks){
    for (int i = 0; i < networks.size(); i++) {
        Network net = *networks[i];
        int s = networks[i]->getCodeA()->getCode();
        int s2 = networks[i]->getCodeB()->getCode();
        if ((s == name->getCode() && s2==name2->getCode()) || (s2 == name->getCode() && s == name2->getCode()))
            return networks[i];
    }
    return NULL;
}

vector<string> split(const string &str, const char del) { // del= delimitador
    vector<string> splitted;
    string tmp;
    for(char i : str) {
        if(i!=del) {
            tmp += i;
        } else {
            splitted.push_back(tmp);
            tmp = "";
        }
    }

    splitted.push_back(tmp);

    return splitted;
}

vector<Network *> readNetworks(string selected, vector<Station*> nodes)
{
    string fname = selected + "edges.csv";
    vector<vector<string>> content;
    vector<string> row;
    string line, word;

    vector<Network *> networks;

    fstream file (fname, ios::in);
    if(file.is_open())
    {
        while(getline(file, line))
        {
            row.clear();

            stringstream str(line);

            while (std::getline(file, line)) {
                if (line.empty()) continue;
                vector<string> row = split(line, ',');
                Station *src = findNode(stoi(row[0]), nodes);
                Station *dest = findNode(stoi(row[1]), nodes);
                auto *net = new Network(src, dest, stod(row[2]));
                g.addEdge(src, dest, net->getWeight());
                g.addEdge(dest, src, net->getWeight());
                networks.push_back(net);
            }
        }
    }
    else
        cout<<"Could not open the file\n";

    return networks;
}