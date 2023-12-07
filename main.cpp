#include <sstream>
#include <iostream>
#include <stdlib.h>  
#include <stdio.h> 
#include "graph.h"

int main(int argc, char* argv[]) {
    
    // Check if the required command-line arguments are provided
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " <graph_filename> <direction>" << std::endl;
        return 1;
    }

    // Extract graph filename and direction from command-line arguments
    std::string graphFilename = argv[1];
    std::string direction = argv[2];

    // Create an instance of the Graph class
    Graph* graph = new Graph();

    // Load the graph from the specified file
    if (!graph->loadGraph(graphFilename, direction)) {
        std::cout << "Failed to load the graph." << std::endl;
        delete graph;
        return 1;
    }

    std::string query;
    while (true) {

        // Read user query
        std::cin >> query;

        if (query == "find") {
            // Process a query to find a path
            int source, destination, flag;
            std::cin >> source >> destination >> flag;
            graph->runDijkstra(source, destination, flag);
        } else if (query == "write") {
            // Process a query to write the path
            std::string action;
            int source, destination;
            std::cin >> action >> source >> destination;
            graph->writePath(source, destination);
        } else if (query == "stop") {
            // Exit the program
            delete graph;
            return 0;
        } else {
            // Invalid query, print an error message
            std::cout << "Invalid input, please try again" << std::endl;
        }
    }
}
