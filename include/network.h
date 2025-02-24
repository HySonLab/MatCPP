#ifndef __NETWORK__
#define __NETWORK__

#include <vector>
#include <limits>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

const double epsilon = 1e-3;

class Vertex {
public:
    int id;

    Vertex() : id(-1) {}

    Vertex(int id) : id(id) {}

    Vertex(const Vertex& other) : id(other.id) {}

    // Comparison operator for natural order by id
    bool operator<(const Vertex& other) const {
        return id < other.id;
    }

    bool operator==(const Vertex& other) const {
        return id == other.id;
    }
};

class Edge {
public:
    Vertex u, v;
    double length, load;
    int id;

    Edge() : length(0.0), load(0.0), id(-1) {}

    Edge(const Vertex& u, const Vertex& v, double length, double load, int id)
        : u(u), v(v), length(length), load(load), id(id) {}

    Edge(const Edge& other)
        : u(other.u), v(other.v), length(other.length), load(other.load), id(other.id) {}

    // Comparison operator for natural order by id
    bool operator<(const Edge& other) const {
        return id < other.id;
    }

    bool operator==(const Edge& other) const {
        return id == other.id;
    }

    // Converts Edge to a string representation
    std::string toString() const {
        std::ostringstream oss;
        oss << "Edge(ID: " << id 
            << ", u: " << u.id 
            << ", v: " << v.id 
            << ", length: " << length 
            << ", load: " << load << ")";
        return oss.str();
    }
};

class Network {
public:
    std::vector<Edge> edges;
    std::vector<Edge> deliver_edges;
    std::vector<Vertex> vertices;
    double vehicle_curb_weight, total_load;
    Vertex depot;

    // Adjacency matrix for shortest paths
    std::vector<std::vector<double>> dist;

    // Map to store the edges on the shortest path between two vertices
    std::map<std::pair<int, int>, std::vector<Edge>> shortest_paths;

    // Default Constructor
    Network()
        : vehicle_curb_weight(0.0), total_load(0.0) {}

    // Parameterized Constructor
    Network(const std::vector<Edge>& edges, const std::vector<Edge>& deliver_edges, const std::vector<Vertex>& vertices,
            double vehicle_curb_weight, double total_load, const Vertex& depot)
        : edges(edges), deliver_edges(deliver_edges), vertices(vertices), vehicle_curb_weight(vehicle_curb_weight),
          total_load(total_load), depot(depot) {}

    // Copy Constructor
    Network(const Network& other)
        : edges(other.edges), deliver_edges(other.deliver_edges), vertices(other.vertices),
          vehicle_curb_weight(other.vehicle_curb_weight),
          total_load(other.total_load), depot(other.depot), dist(other.dist), shortest_paths(other.shortest_paths) {}

    void floyd_warshall() {
        int n = vertices.size();
        dist.assign(n, std::vector<double>(n, std::numeric_limits<double>::infinity()));

        // Initialize distances with edge lengths and the shortest path edges
        for (const auto& edge : edges) {
            int u = edge.u.id;
            int v = edge.v.id;
            if (edge.length < dist[u][v]) {
                dist[u][v] = edge.length;
                dist[v][u] = edge.length; // Assuming undirected graph
                shortest_paths[{u, v}] = {edge};  // Direct edge from u to v
                shortest_paths[{v, u}] = {edge};  // Assuming undirected graph
            }
        }

        // Distance to self is zero
        for (int i = 0; i < n; ++i) {
            dist[i][i] = 0.0;
        }

        // Floyd-Warshall Algorithm
        for (int k = 0; k < n; ++k) {
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    if (dist[i][k] < std::numeric_limits<double>::infinity() &&
                        dist[k][j] < std::numeric_limits<double>::infinity()) {
                        double new_dist = dist[i][k] + dist[k][j];
                        if (new_dist < dist[i][j]) {
                            dist[i][j] = new_dist;
                            // Update the shortest path
                            shortest_paths[{i, j}] = shortest_paths[{i, k}];
                            shortest_paths[{i, j}].insert(shortest_paths[{i, j}].end(),
                                                           shortest_paths[{k, j}].begin(),
                                                           shortest_paths[{k, j}].end());
                        }
                    }
                }
            }
        }
    }

    void print() const {
        std::cout << "Network Information:\n";

        // Print Depot
        std::cout << "Depot: Vertex ID = " << depot.id << "\n";

        // Print Vertices
        std::cout << "Vertices:\n";
        for (const auto& vertex : vertices) {
            std::cout << "Vertex ID = " << vertex.id << "\n";
        }

        // Print Edges
        std::cout << "Edges:\n";
        for (const auto& edge : edges) {
            std::cout << "Edge ID = " << edge.id << ", ";
            std::cout << "From Vertex " << edge.u.id << " to Vertex " << edge.v.id << ", ";
            std::cout << "Length = " << edge.length << ", Load = " << edge.load << "\n";
        }

        // Print Vehicle Curb Weight and Total Load
        std::cout << "Vehicle Curb Weight = " << vehicle_curb_weight << "\n";
        std::cout << "Total Load = " << total_load << std::endl;
    }

    // Function to print the edges on the shortest path between two vertices
    void print_shortest_path(int start, int end) const {
        auto key = std::make_pair(start, end);
        if (shortest_paths.find(key) != shortest_paths.end()) {
            std::cout << "Shortest path from Vertex " << start << " to Vertex " << end << ":\n";
            for (const auto& edge : shortest_paths.at(key)) {
                std::cout << "Edge ID = " << edge.id << ", ";
                std::cout << "From Vertex " << edge.u.id << " to Vertex " << edge.v.id << ", ";
                std::cout << "Length = " << edge.length << "\n";
            }
        } else {
            std::cout << "No path found from Vertex " << start << " to Vertex " << end << "\n";
        }
    }
};

struct Solution {
    std::vector<Edge> edges;
    std::vector<bool> direction; // true means reversal is needed
    Network network;
    double cost;

    void print() const {
        std::cout << "Solution:\n";
        std::cout << "Edges:\n";
        for (size_t i = 0; i < edges.size(); ++i) {
            std::cout << "  " << edges[i].toString();
            if (i < direction.size()) {
                std::cout << (direction[i] ? " (reversed)" : " (forward)");
            }
            std::cout << "\n";
        }
        std::cout << "Cost: " << cost << "\n";
    }
};

#endif