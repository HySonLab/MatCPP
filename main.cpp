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
    auto solution = new_greedy_constructive_heuristics(network);
    solution = multiple_shift_operator(network, solution);
    solution = reorder_edges(network, solution);
    solution = matheuristics(network, solution, network.edges.size() / 2, 5, 2);
    auto end = std::chrono::high_resolution_clock::now();
    auto runtime = std::chrono::duration<double>(end - start).count();
    return {solution, runtime};
}

void save_result_to_csv(const std::string &output_file, const std::string &result, bool write_header) {
    std::ofstream file(output_file, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Failed to open output file: " << output_file << std::endl;
        return;
    }

    // Write header only if the file is empty or it's the first write
    if (write_header) {
        file << "Name,Number of nodes,Number of edges,Number of deliver edges,Cost,Time (ms),Time (s)\n";
    }

    file << result << "\n";
    file.close();
}

int main(int argc, char* argv[]) {
    auto program_start = std::chrono::high_resolution_clock::now();

    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <folder_path> <output_csv>" << std::endl;
        return 1;
    }

    std::string folder_path = argv[1];
    std::string output_csv = argv[2];

    bool first_write = true;
    for (const auto &entry : std::filesystem::directory_iterator(folder_path)) {
        if (entry.is_regular_file() && entry.path().extension() == ".txt") {
            std::string filename = entry.path().string();
            std::cout << "Processing file: " << filename << std::endl;

            auto [solution, runtime] = run_matheuristics(filename);

            size_t num_nodes = solution.network.vertices.size();
            size_t num_edges = solution.network.edges.size();
            size_t num_deliver_edges = solution.edges.size();
            double cost = solution.cost;
            double runtime_ms = runtime * 1000;

            std::string result = entry.path().filename().string() + "," +
                                 std::to_string(num_nodes) + "," +
                                 std::to_string(num_edges) + "," +
                                 std::to_string(num_deliver_edges) + "," +
                                 std::to_string(cost) + "," +
                                 std::to_string(runtime_ms) + "," +
                                 std::to_string(runtime);

            save_result_to_csv(output_csv, result, first_write);
            first_write = false; // Ensure header is written only once
            std::cout << "The runtime for file " << filename << " is " << runtime_ms << "ms" << std::endl;
        }
    }

    auto program_end = std::chrono::high_resolution_clock::now();
    auto total_runtime = std::chrono::duration<double>(program_end - program_start).count();
    std::cout << "Total runtime: " << total_runtime << " seconds" << std::endl;

    return 0;
}
