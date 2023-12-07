#include "graph.h"
#include <bits/stdc++.h>
#include "minheap.h"

using namespace std;
const double DOUBLE_MAX = 99999999.0;

Graph::Graph() {
    // Initialize graph properties
    numVertices = 0;
    numEdges = 0;
    isDirected = false;
    source = -1;
    graphTraversed = false;
    fullTraversal = false;

    // Initialize arrays to nullptr
    adjacencyLists = nullptr;
    extractedVertices = nullptr;
    relaxedVertices = nullptr;
    predecessor = nullptr;
    distance = nullptr;
    initializeArrays(); // Call the function to initialize arrays
}

Graph::~Graph() {
    // Deallocate memory for arrays
    delete[] extractedVertices;
    delete[] relaxedVertices;
    delete[] predecessor;
    delete[] distance;

    for (int v = 0; v < numVertices; v++) {
        delete[] adjacencyLists[v];
    }
    delete[] adjacencyLists;
}

bool Graph::loadGraph(const string& filename, const string& direction) {
    ifstream file(filename);
    if (!file.is_open()) {
        cout << "Failed to open the graph file." << endl;
        return false;
    }

    // Read the first line to get the number of vertices and edges
    int n, m;
    if (!(file >> n >> m)) {
        cout << "Invalid format for the first line in the input file." << endl;
        return false;
    }
    numVertices = n + 1; // Increment numVertices to include node 0
    numEdges = m;

    // Initialize arrays and data structures
    initializeArrays();

    // Skip the rest of the first line
    file.ignore(numeric_limits<streamsize>::max(), '\n');

    // Read edges from the file
    int edgeId, startNode, endNode;
    double weight;
    while (file >> edgeId >> startNode >> endNode >> weight) {
        // Increment node indices
        startNode++;
        endNode++;

        // Check for invalid node IDs
        if (startNode > numVertices || endNode > numVertices || startNode < 1 || endNode < 1) {
            cout << "Invalid node IDs in the input file." << endl;
            return false;
        }

        // Create edge and update adjacency lists
        addEdge(startNode, endNode, weight);

        // If undirected, add reverse edge
        if (direction == "undirected") {
            addEdge(endNode, startNode, weight);
        }
    }

    // Set graph direction
    isDirected = (direction == "directed");

    file.close();
    return true;
}

void Graph::initializeArrays() {
    // Initialize arrays and data structures
    adjacencyLists = new Edge*[numVertices];
    for (int i = 0; i < numVertices; ++i) {
        adjacencyLists[i] = nullptr;
    }

    extractedVertices = new double[numVertices](); // Initialized to 0.0
    relaxedVertices = new double[numVertices]();   // Initialized to 0.0
    predecessor = new int[numVertices]();           // Initialized to 0
    distance = new double[numVertices]();           // Initialized to DOUBLE_MAX

    for (int i = 0; i < numVertices; ++i) {
        distance[i] = DOUBLE_MAX;
    }
}

void Graph::addEdge(int startNode, int endNode, double weight) {
    // Create a new Edge instance to represent the edge
    Edge* edge = new Edge;
    edge->destination = endNode;  // Set the destination vertex of the edge
    edge->weight = weight;         // Set the weight of the edge

    // If the adjacency list for the startNode is not initialized, allocate memory for it
    if (adjacencyLists[startNode] == nullptr) {
        adjacencyLists[startNode] = new Edge[numEdges];
    }

    int j = 0;
    // Find the first available slot in the adjacency list for the new edge
    while (adjacencyLists[startNode][j].destination != 0) {
        j++;
    }

    // Copy the edge information into the adjacency list
    adjacencyLists[startNode][j] = *edge;

    // Deallocate memory for the temporary edge instance
    delete edge;
}


void Graph::runDijkstra(int newSource, int destination, int flag) {
    // Initialize variables and data structures
    int updatedSource = newSource;  // Set the updated source vertex
    int tracker = numVertices;      // Track the number of vertices
    bool dijkstraCompleted = true;  // Flag to track if Dijkstra's algorithm completed successfully
    bool partialTraversal = true;   // Flag to track if a partial traversal occurred

    // Initialize vectors and arrays for Dijkstra's algorithm
    vector<bool> extracted(tracker, false);
    fill(extractedVertices, extractedVertices + tracker, -1.0);
    fill(relaxedVertices, relaxedVertices + tracker, -1.0);
    fill(predecessor, predecessor + tracker, -1);
    fill(distance, distance + tracker, DOUBLE_MAX);

    extracted[updatedSource] = true;  // Mark the updated source vertex as extracted

    MinHeap minHeap(tracker);         // Create a MinHeap for efficient priority queue operations
    distance[updatedSource] = 0;      // Set the distance of the updated source vertex to 0
    minHeap.push(0, updatedSource);   // Push the updated source vertex into the MinHeap

    // Print the insertion of the updated source vertex if flag is set
    if (flag == 1) {
        cout << std::fixed << std::setprecision(4) << "Insert vertex " << updatedSource << ", key=" << setw(12) << distance[updatedSource] << endl;
    }

    // Main loop of Dijkstra's algorithm
    while (!minHeap.empty()) {
        int extractedVertex = minHeap.pop();  // Extract the vertex with the minimum distance

        // Mark the vertex as extracted and update extractedVertices array
        extracted[extractedVertex] = true;
        extractedVertices[extractedVertex] = distance[extractedVertex];

        // Print the deletion of the extracted vertex if flag is set
        if (flag == 1) {
            cout << std::fixed << std::setprecision(4) << "Delete vertex " << extractedVertex << ", key=" << setw(12) << distance[extractedVertex] << endl;
        }

        // Break the loop if the destination vertex is reached
        if (extractedVertex == destination) {
            break;
        }

        // Explore neighbors of the extracted vertex
        if (adjacencyLists[extractedVertex] != nullptr) {
            int edgeIndex = 0;
            while (adjacencyLists[extractedVertex][edgeIndex].destination != 0) {

                int neighborVertex = adjacencyLists[extractedVertex][edgeIndex].destination;
                double edgeWeight = adjacencyLists[extractedVertex][edgeIndex].weight;

                // Update distance and other arrays if a shorter path is found
                if (!extracted[neighborVertex] && distance[extractedVertex] + edgeWeight < distance[neighborVertex]) {

                    double oldPathWeight = distance[neighborVertex];
                    distance[neighborVertex] = distance[extractedVertex] + edgeWeight;
                    predecessor[neighborVertex] = extractedVertex;
                    relaxedVertices[neighborVertex] = distance[neighborVertex];

                    // Print the decrease key operation if flag is set
                    if (oldPathWeight != DOUBLE_MAX && flag == 1) {
                        cout << "Decrease key of vertex " << neighborVertex << ", from " << setw(12) << oldPathWeight << " to " << std::fixed << std::setprecision(4) << setw(12) << distance[neighborVertex] << endl;
                    }

                    minHeap.push(distance[neighborVertex], neighborVertex);

                    // Print the insertion of the neighbor vertex if flag is set
                    if (flag == 1) {
                        cout << std::fixed << std::setprecision(4) << "Insert vertex " << neighborVertex << ", key=" << setw(12) << distance[neighborVertex] << endl;
                    }
                }
                edgeIndex++;
            }
        }
    }

    // Process any remaining vertices in the MinHeap
    while (!minHeap.empty()) {
        int remainingVertex = minHeap.pop();
        if (!extracted[remainingVertex]) {
            partialTraversal = false;
        }
    }

    // Update traversal flags based on the completion of Dijkstra's algorithm
    if (!partialTraversal) {
        dijkstraCompleted = false;
    }

    fullTraversal = dijkstraCompleted;
}


void Graph::writePath(int s, int d) {
    // Check if path computation has been performed
    if (!graphTraversed) {
        cout << "Error: No path computation done." << endl;
        return;
    }

    // Check for a valid source-destination pair
    if (s != source || d < 1 || d >= numVertices) {
        cout << "Error: Invalid source-destination pair." << endl;
        return;
    }

    // Check if an s-d path is computed
    if (extractedVertices[d] != -1.0 || relaxedVertices[d] != -1.0) {
        // Create an array to store the path
        int* path = (int*)malloc(numVertices * sizeof(int));
        int current = d;
        int pathSize = 0;

        // Reconstruct the path
        while (current != s) {
            path[pathSize++] = current;
            current = predecessor[current];
        }

        path[pathSize] = s;

        // Determine if the path is known to be the shortest
        bool isShortest = (extractedVertices[d] != -1.0);

        // Print the path information
        cout << "Path: ";
        for (int i = pathSize; i >= 0; i--) {
            cout << path[i] << " ";
        }
        cout << endl;

        // Print whether the path is known to be the shortest or not
        cout << (isShortest ? "Type: Shortest path" : "Type: Path not known to be shortest") << endl;

        // Print the path weight
        cout << "Path weight: " << std::fixed << std::setprecision(4) << setw(13) << distance[d] << endl;

        // Deallocate memory for the path array
        free(path);
    } else if (!fullTraversal) {
        cout << "No path between " << s << " and " << d << " has been computed yet." << endl;
    } else {
        cout << "No path exists between " << s << " and " << d << "." << endl;
    }
}


void Graph::printAdjacencyLists() {
    // Iterate over each vertex in the graph
    for (int v = 0; v < numVertices; v++) {
        // Print the header for the current vertex's adjacency list
        cout << "Adjacency list for vertex " << v << ": ";

        // Check if the adjacency list for the current vertex is not empty
        if (adjacencyLists[v] != nullptr) {
            int j = 0;

            // Iterate over the edges in the adjacency list until the sentinel value 0 is encountered
            while (adjacencyLists[v][j].destination != 0) {
                // Print the destination vertex and weight of the current edge
                cout << "(" << adjacencyLists[v][j].destination << ", " << adjacencyLists[v][j].weight << ") ";

                // Move to the next edge in the adjacency list
                j++;
            }
        }

        // Print the predecessor information for the current vertex
        cout << "Predecessor: " << predecessor[v];

        // Move to the next line for the next vertex
        cout << endl;
    }
}
