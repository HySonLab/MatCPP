#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <string>
#include <filesystem>

#include "utils.h"
#include "heuristics.h"

std::pair<Solution, double> run_new_greedy_heuristics(const std::string &filename) {
    auto start = std::chrono::high_resolution_clock::now();
    auto network = read_network(filename);
    network.floyd_warshall();
    auto solution = new_greedy_constructive_heuristics(network);
    auto end = std::chrono::high_resolution_clock::now();
    auto runtime = std::chrono::duration<double>(end - start).count();
    return {solution, runtime};
}

void append_result_to_csv(const std::string &output_file, const std::string &header, const std::string &line, bool write_header) {
    std::ofstream file(output_file, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }

    // Write header only if required
    if (write_header) {
        file << header << "\n";
    }

    // Write the result line
    file << line << "\n";
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

    // Define CSV header
    const std::string header = "Name,Number of nodes,Number of edges,Number of deliver edges,Cost,Time (ms),Time (s)";

    // Check if file exists to determine if we need to write the header
    bool file_exists = std::filesystem::exists(output_csv);

    for (const auto &entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            std::string filename = entry.path().string();
            std::cout << "Processing file: " << filename << std::endl;

            auto [solution, runtime] = run_new_greedy_heuristics(filename);

            // Collect data for CSV
            size_t num_nodes = solution.network.vertices.size();
            size_t num_edges = solution.network.edges.size();
            size_t num_deliver_edges = solution.edges.size();
            double cost = solution.cost;
            double runtime_ms = runtime * 1000; // Convert to milliseconds

            std::string result_line = entry.path().filename().string() + "," +
                                      std::to_string(num_nodes) + "," +
                                      std::to_string(num_edges) + "," +
                                      std::to_string(num_deliver_edges) + "," +
                                      std::to_string(cost) + "," +
                                      std::to_string(runtime_ms) + "," +
                                      std::to_string(runtime);

            // Write result immediately to CSV
            append_result_to_csv(output_csv, header, result_line, !file_exists);
            file_exists = true; // Ensure header is written only once
        }
    }

    auto program_end = std::chrono::high_resolution_clock::now();
    auto total_runtime = std::chrono::duration<double>(program_end - program_start).count();
    std::cout << "Total runtime: " << total_runtime << " seconds" << std::endl;

    return 0;
}
