#ifndef __MATHEURISTICS__
#define __MATHEURISTICS__

#include "milp.h"
#include "heuristics.h"

void set_xy_between_periods(MILPSolution &milp_values, Network network, const Vertex& u, const Vertex& v, const Vertex& next_u, int k) {
    milp_values.y[u][v][k] = 1;
    auto path = network.shortest_paths[{v.id, next_u.id}];
    auto curu = v;
    for(const auto& edge : path) {
        auto curv = edge.u == curu? edge.v : edge.u;
        milp_values.x[curu][curv][k] = 1;
        curu = curv;
    }
    my_assert(curu == next_u, "They must be the same");
}

MILPSolution convert_tour_to_milp_solution(Solution solution) {
    // Return the x_values and y_values corresponding to the tour in solution
    MILPSolution milp_values;
    auto u = solution.direction.back()? solution.edges.back().v : solution.edges.back().u;
    auto v = solution.direction.back()? solution.edges.back().u : solution.edges.back().v;
    auto next_u = solution.network.depot;
    auto m = solution.edges.size();
    set_xy_between_periods(milp_values, solution.network, u, v, next_u, m - 1);
    for (int i = 0; i < m - 1; i++) {
        auto edge = solution.edges[i];
        u = solution.direction[i]? edge.v : edge.u;
        v = solution.direction[i]? edge.u : edge.v;
        next_u = solution.direction[i + 1]? solution.edges[i + 1].v : solution.edges[i + 1].u;
        set_xy_between_periods(milp_values, solution.network, u, v, next_u, i);
    }
    milp_values.network = solution.network;
    milp_values.objective = solution.cost;
    return milp_values;
}

Solution convert_milp_solution_to_tour(const Network& network, const MILPSolution& milp_solution) {
    if (milp_solution.y.empty()) {
        std::cout << "Milp solution is empty" << std::endl;
        return Solution();
    }
    Solution solution;
    solution.network = network;
    int m = network.deliver_edges.size();
    solution.direction.resize(m, false);
    auto x = milp_solution.x;
    auto y = milp_solution.y;
    for (int k = 0; k < m; k++) {
        for (const auto& edge : network.deliver_edges) {
            if (y[edge.u][edge.v][k] + y[edge.v][edge.u][k] > 0) {
                my_assert(std::find(solution.edges.begin(), solution.edges.end(), edge) == solution.edges.end(), "Duplicate edge found!");
                solution.edges.push_back(edge);
                if (y[edge.v][edge.u][k] > 0) {solution.direction[k] = true;}
                break;
            }
        }
    }
    my_assert(solution.edges.size() == (size_t) m, "Edge count mismatch!");
    solution.cost = milp_solution.objective;
    auto dp_sol = cost_tour(network, solution.edges);
    my_assert(dp_sol.cost <= solution.cost + eps, "Model validation failed! DP cost: " + std::to_string(dp_sol.cost) + " Solution cost: " + std::to_string(solution.cost));
    return dp_sol;
}
 
MILPSolution block_operator(Network network, const MILPSolution& milp_solution, int start, int block_size) {
    ArcRoutingFormulation model(network);
    int end = start + block_size;
    int m = network.deliver_edges.size();
    my_assert(start >= 0 && start < m, "Start index out of bounds. start: " + std::to_string(start) + " m = " + std::to_string(m));

    if (end < m) {
        model.fix_x(end, m - 1, milp_solution.x);
        model.fix_y(end, m - 1, milp_solution.y);
        model.fix_x(0, start - 2, milp_solution.x);
        model.fix_y(0, start - 1, milp_solution.y);
    }
    else {
        model.fix_x(end % m, start - 2, milp_solution.x);
        model.fix_y(end % m, start - 1, milp_solution.y);
    }
    model.limit_objective(milp_solution.objective);
    return model.solve();
}

Solution matheuristics(const Network& network, Solution solution, int block_size, int iterations, int num_tries) {
    my_assert(0 < (int) network.deliver_edges.size(), "The number of delivery edges must be positive!");

    auto milp_values = convert_tour_to_milp_solution(solution); // Solution and milp values are always the same
    std::cout << "The number of edges is " << network.deliver_edges.size() << std::endl;

    // Statistics for the solver
    int num_sucesses = 0;
    int num_run = 0;

    for (int i = 0; i < iterations; i++) {
        auto start_time = std::chrono::high_resolution_clock::now();
        solution = multiple_shift_operator(network, solution);
        auto end_time = std::chrono::high_resolution_clock::now();
        std::cout << "multiple_shift_operator took " << std::chrono::duration<double>(end_time - start_time).count() << " seconds" << std::endl;

        start_time = std::chrono::high_resolution_clock::now();
        solution = reorder_edges(network, solution);
        end_time = std::chrono::high_resolution_clock::now();
        std::cout << "reorder_edges took " << std::chrono::duration<double>(end_time - start_time).count() << " seconds" << std::endl;

        start_time = std::chrono::high_resolution_clock::now();
        milp_values = convert_tour_to_milp_solution(solution);
        end_time = std::chrono::high_resolution_clock::now();
        std::cout << "convert_tour_to_milp_solution took " << std::chrono::duration<double>(end_time - start_time).count() << " seconds" << std::endl;

        for (int j = 0; j < num_tries; j++) {
            auto block_start = get_random_int(network.deliver_edges.size());
            std::cout << "Block chosen to be: [" << block_start << ", " << block_start + block_size << "]" << std::endl;
            auto current_obj = milp_values.objective;

            start_time = std::chrono::high_resolution_clock::now();
            auto temp = block_operator(network, milp_values, block_start, block_size);
            end_time = std::chrono::high_resolution_clock::now();
            std::cout << "block_operator took " << std::chrono::duration<double>(end_time - start_time).count() << " seconds" << std::endl;

            bool solvable = !temp.y.empty() && temp.objective < (current_obj - epsilon);
            num_run++;

            if (solvable) {
                num_sucesses++;
                std::cout << "There is improvement" << std::endl;
                milp_values = temp;
                start_time = std::chrono::high_resolution_clock::now();
                solution = convert_milp_solution_to_tour(network, milp_values);
                end_time = std::chrono::high_resolution_clock::now();
                std::cout << "convert_milp_solution_to_tour took " << std::chrono::duration<double>(end_time - start_time).count() << " seconds" << std::endl;
                break;
            }
            std::cout << "There is no improvement" << std::endl;
        }
    }

    std::cout << "Number of successes / runs: " << num_sucesses << " / " << num_run << std::endl;
    return convert_milp_solution_to_tour(network, milp_values);
}

Solution arc_routing_formulation(const Network& network) {
    ArcRoutingFormulation model(network);
    auto milp_values = model.solve();
    return convert_milp_solution_to_tour(network, milp_values);
}

#endif