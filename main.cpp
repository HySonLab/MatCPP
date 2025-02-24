#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>

#include "milp.h"
#include "utils.h"
#include "heuristics.h"
#include "matheuristics.h"

std::pair<Solution, double> run_matheuristics(const std::string &filename) {
    auto start = std::chrono::high_resolution_clock::now();
    auto network = read_network(filename);
    network.floyd_warshall();
    auto solution = greedy_constructive_heuristics(network);
    solution = multiple_shift_operator(network, solution);
    solution = reorder_edges(network, solution);
    solution = matheuristics(network, solution, network.deliver_edges.size() / 2, 5, 10);
    auto end = std::chrono::high_resolution_clock::now();
    auto runtime = std::chrono::duration<double>(end - start).count();
    return {solution, runtime};
}

void save_results_to_csv(const std::string &output_file, const std::vector<std::string> &results) {
    std::ofstream file(output_file);
    if (!file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }

    // Write header
    file << "Name,Number of nodes,Number of edges,Number of deliver edges,Cost,Time (ms),Time (s)\n";

    // Write results
    for (const auto &line : results) {
        file << line << "\n";
    }

    file.close();
}

int main(int argc, char* argv[]) {
    // Start timing for the entire program
    auto program_start = std::chrono::high_resolution_clock::now();

    // Check if a folder path is provided
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <folder_path> <output_csv>" << std::endl;
        return 1;
    }

    // Get the folder path and output file from command-line arguments
    std::string folder_path = argv[1];
    std::string output_csv = argv[2];

    std::vector<std::string> results;

    for (const auto &entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            std::string filename = entry.path().string();
            std::cout << "Processing file: " << filename << std::endl;

            auto [solution, runtime] = run_matheuristics(filename);

            // Collect data for CSV
            size_t num_nodes = solution.network.vertices.size();
            size_t num_edges = solution.network.edges.size();
            size_t num_deliver_edges = solution.edges.size();
            double cost = solution.cost;
            double runtime_ms = runtime * 1000; // Convert to milliseconds

            results.push_back(entry.path().filename().string() + "," +
                              std::to_string(num_nodes) + "," +
                              std::to_string(num_edges) + "," +
                              std::to_string(num_deliver_edges) + "," +
                              std::to_string(cost) + "," +
                              std::to_string(runtime_ms) + "," +
                              std::to_string(runtime));
        }
    }

    // Save results to CSV
    save_results_to_csv(output_csv, results);

    auto program_end = std::chrono::high_resolution_clock::now();
    auto total_runtime = std::chrono::duration<double>(program_end - program_start).count();
    std::cout << "Total runtime: " << total_runtime << " seconds" << std::endl;

    return 0;
}
