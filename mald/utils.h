#ifndef __UTILS__
#define __UTILS__

#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cassert>
#include <random>
#include <numeric>
#include <set>
#include <stack>
#include <unordered_set>

#include "network.h"

void my_assert(bool condition, const std::string& message) {
    if (!condition) {
        std::cout << "Assertion failed: " << message << std::endl;
        exit(-1);
    }
}

std::vector<std::string> parse_line(const std::string& str, char separator = ' ') {
    std::vector<std::string> result;
    std::string word;
    std::istringstream stream(str);

    while (std::getline(stream, word, separator)) {
        if (!word.empty()) { // Avoid adding empty substrings
            result.push_back(word);
        }
    }

    return result;
}

bool is_graph_connected(const Network& network) {
    if (network.vertices.empty()) {
        return true; // An empty graph is trivially connected.
    }

    // Keep track of visited vertices
    std::unordered_set<int> visited;

    // Stack for DFS
    std::stack<int> stack;

    // Start DFS from the first vertex
    stack.push(network.vertices[0].id);
    visited.insert(network.vertices[0].id);

    while (!stack.empty()) {
        int current = stack.top();
        stack.pop();

        // Explore neighbors of the current vertex
        for (const auto& edge : network.edges) {
            if (edge.u.id == current && visited.find(edge.v.id) == visited.end()) {
                stack.push(edge.v.id);
                visited.insert(edge.v.id);
            } else if (edge.v.id == current && visited.find(edge.u.id) == visited.end()) {
                stack.push(edge.u.id);
                visited.insert(edge.u.id);
            }
        }
    }

    // Check if all vertices were visited
    return visited.size() == network.vertices.size();
}

void check_depot_deliver(Network &network) {
    bool depot_delivered = false;
    for (const auto& edge : network.deliver_edges) {
        if (edge.v == network.depot || edge.u == network.depot) {
            depot_delivered = true;
        }
    }
    if (!depot_delivered) {
        // Add a dummy edge so that we do not break the formulation
        Edge edge(network.depot, network.depot, 0, 0, network.edges.size());
        network.deliver_edges.push_back(edge);
        network.edges.push_back(edge);
    }
}

Network read_network(std::string filename) {
    std::ifstream file;
    file.open(filename);

    std::string str;
    getline(file, str);
    getline(file, str);
    auto num_nodes = std::stoi(str);

    getline(file, str);
    getline(file, str);
    auto num_edges = std::stoi(str);

    getline(file, str);
    getline(file, str);

    Network network;
    network.vehicle_curb_weight = std::stof(str);

    std::vector<Edge> edges;
    std::vector<Edge> deliver_edges;

    getline(file, str);
    for (int i = 0; i < num_edges; ++i) {
        getline(file, str);
        std::vector<std::string> words = parse_line(str);
        assert(words.size() == 4);
        const int first = stoi(words[0]);
        const int second = stoi(words[1]);
        const double d = stof(words[2]);
        const double q = stof(words[3]);
        edges.push_back(Edge(Vertex(first), Vertex(second), d, q, i));
        if (q > 0.0) {
            deliver_edges.push_back(Edge(Vertex(first), Vertex(second), d, q, i));
        }
        network.total_load += q;
    }

    std::set<std::pair<int, int> > visited;
    for (auto edge : edges) {
        if (visited.count({edge.v.id, edge.u.id}) > 0) {
            my_assert(false, "Edge already exists");
        }
        visited.insert({edge.v.id, edge.u.id});
        visited.insert({edge.u.id, edge.v.id});
    }

    file.close();

    std::vector<Vertex> vertices;
    for (int i = 0; i < num_nodes; i++) {
        vertices.push_back(Vertex(i));
    }

    network.vertices = vertices;
    network.edges = edges;
    network.deliver_edges = deliver_edges;
    network.depot = vertices[0];

    my_assert(0 < (int) network.deliver_edges.size(), "The number of edges must be greater than 0.");
    my_assert(is_graph_connected(network), "The graph must be connected.");
    check_depot_deliver(network);
 
    return network;
}

// Function to generate a random integer between 0 and n - 1
int get_random_int(int n) {
    if (n <= 0) {
        throw std::invalid_argument("n must be greater than 0");
    }

    static std::random_device rd; // Static to initialize only once
    static std::mt19937 gen(rd()); // Static for performance and consistency
    std::uniform_int_distribution<> dist(0, n - 1);

    return dist(gen);
}

std::vector<int> generate_random_indices(int size, int max_value) {
    std::vector<int> indices(max_value);
    std::iota(indices.begin(), indices.end(), 0); // Fill with 0, 1, ..., max_value-1

    std::random_device rd;
    std::mt19937 rng(rd());

    // Shuffle the indices and pick the first 'size' elements
    std::shuffle(indices.begin(), indices.end(), rng);
    return std::vector<int>(indices.begin(), indices.begin() + size);
}

#endif