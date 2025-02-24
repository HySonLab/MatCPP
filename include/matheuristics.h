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
 
MILPSolution block_operator(ArcRoutingFormulation model, const MILPSolution& milp_solution, int start, int block_size) {
    auto network = model.get_network();
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
    model.limit_objective(milp_solution.objective - 1);
    return model.solve();
}

Solution matheuristics(const Network& network, Solution solution, int block_size, int iterations, int num_tries) {
    my_assert(0 < (int) network.deliver_edges.size(), "The number of delivery edges must be positive!");
    ArcRoutingFormulation model(network);
    auto milp_values = convert_tour_to_milp_solution(solution); // Solution and milp values are always the same
    std::cout << "The number of edges is " << network.deliver_edges.size() << std::endl;

    for (int i = 0; i < iterations; i++) {
        // Do traditional heuristics optimizations
        solution = find_best_reversal(network, solution);
        solution = multiple_shift_operator(network, solution);
        solution = reorder_edges(network, solution);
        milp_values = convert_tour_to_milp_solution(solution);

        for (int j = 0; j < num_tries; j++) {
            // Do the block operator
            auto start = get_random_int(network.deliver_edges.size());
            std::cout << "Block chosen to be: [" << start << ", " << start + block_size << "]" << std::endl;
            auto current_obj = milp_values.objective;
            auto temp = block_operator(model, milp_values, start, block_size);
            bool solvable = !temp.y.empty() && temp.objective < (current_obj - epsilon);

            // Keep the results for the next iteration
            if (solvable) {
                std::cout << "There is improvement" << std::endl;
                milp_values = temp;
                solution = convert_milp_solution_to_tour(network, milp_values);
                break;
            }
            std::cout << "There is no improvement" << std::endl;
        }
        model.clear_auxiliary_constraints();
    }
    return convert_milp_solution_to_tour(network, milp_values);
}

Solution arc_routing_formulation(const Network& network) {
    ArcRoutingFormulation model(network);
    auto milp_values = model.solve();
    return convert_milp_solution_to_tour(network, milp_values);
}

#endif