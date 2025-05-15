#ifndef __HEURISTICS__
#define __HEURISTICS__

#include "network.h"
#include <algorithm>
#include <vector>
#include <limits>
#include <random>
#include <set>

Solution cost_tour(Network network, std::vector<Edge> tour) {
    Solution sol;
    // Start node
	auto start_node = network.depot;
	const double W = network.vehicle_curb_weight;
	const int m = tour.size();

	// Initialize
    std::vector< std::vector<double> > f;
    f.clear();
    for (int k = 0; k < m; ++k) {
        std::vector<double> vect;
        vect.clear();
        vect.push_back(std::numeric_limits<double>::infinity());
        vect.push_back(std::numeric_limits<double>::infinity());
        f.push_back(vect);
    }

    std::vector<int> direction;
    direction.clear();
    for (int k = 0; k < m; ++k) {
        direction.push_back(-1);
    }

	// Special case if there is only one edge in the list
	if (m == 1) {
		// Current edge's information
        auto i = tour[0].u;
        auto j = tour[0].v;
        const double q = tour[0].load;
        const double d = tour[0].length;

		const double option_0 = (W + q / 2) * d + (W + q) * network.dist[start_node.id][i.id] + W * network.dist[j.id][start_node.id];
		const double option_1 = (W + q / 2) * d + (W + q) * network.dist[start_node.id][j.id] + W * network.dist[i.id][start_node.id];

		if (option_0 <= option_1) {
			f[0][0] = option_0;
			direction[0] = 0;
		} else {
			f[0][0] = option_1;
			direction[0] = 1;
		}

        sol.edges = tour;
        sol.direction = std::vector<bool>(m, false);
        for (int l = 0; l < m; l++) {sol.direction[l] = direction[l];}
        sol.network = network;
        sol.cost = f[0][0];

		return sol;
	}

	// For tracing
	std::vector< std::vector<int> > choice;
    choice.clear();
    for (int k = 0; k < m; ++k) {
        std::vector<int> vect;
        vect.clear();
        vect.push_back(-1);
        vect.push_back(-1);
        choice.push_back(vect);
    }

	// Compute the Q
	std::vector<double> Q;
	for (int k = 0; k < m; ++k) {
		Q.push_back(0);
	}
	Q[m - 1] = tour[m - 1].load;
	for (int k = m - 2; k >= 0; --k) {
		Q[k] = Q[k + 1] + tour[k].load;
	}

	// Dynamic Programming
	for (int k = m - 1; k >= 0; --k) {
		for (int c = 0; c <= 1; ++c) {
			// Skip this case
			if ((k == 0) && (c == 1)) {
				continue;
			}

			// Current edge's information
			const int i = tour[k].u.id;
			const int j = tour[k].v.id;
			const double q = tour[k].load;
			const double d = tour[k].length;

			// Previous node
			int prev = -1;
			if (k == 0) {
				prev = start_node.id;
			} else {
            	if (c == 0) {
            		// If the previous edge is not flipped
            		prev = tour[k - 1].v.id;
				} else {
                	// If the previous edge is flipped
                	prev = tour[k - 1].u.id;
            	}
			}

			double option_0;
			double option_1;

			if (k == m - 1) {
				// For the last edge
				option_0 = (W + q / 2) * d + (W + q) * network.dist[prev][i] + W * network.dist[j][start_node.id];
				option_1 = (W + q / 2) * d + (W + q) * network.dist[prev][j] + W * network.dist[i][start_node.id];
			} else {
				// For all other edges
				option_0 = (W + Q[k] - q / 2) * d + (W + Q[k]) * network.dist[prev][i] + f[k + 1][0];
               	option_1 = (W + Q[k] - q / 2) * d + (W + Q[k]) * network.dist[prev][j] + f[k + 1][1];
			}

			if (option_0 <= option_1) {
            	f[k][c] = option_0;
                choice[k][c] = 0;
            } else {
                f[k][c] = option_1;
                choice[k][c] = 1;
            }
		}
	}

	// Tracing
	int k = 0;
	int c = 0;
	while (k < m) {
		direction[k] = choice[k][c];
		c = choice[k][c];
		++k;
	}

    sol.edges = tour;
    sol.direction = std::vector<bool>(m, false);
    for (int l = 0; l < m; l++) {sol.direction[l] = direction[l];}
    sol.network = network;
    sol.cost = f[0][0];
    return sol;
}

// Greedy Constructive Heuristics Implementation
Solution greedy_constructive_heuristics(const Network& network) {
    // Step 1: Sort edges by weight (greedy criterion)
    std::vector<Edge> edges = network.deliver_edges;
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.length * a.load > b.length * b.load; // Greedy criterion: greatest length * load
    });

    // Step 2: Construct solution iteratively
    std::vector<Edge> constructed_solution;

    for(int i = 0; i < edges.size(); i++) {
    	auto temp_solution = constructed_solution;
    	auto min_cost = std::numeric_limits<double>::infinity();
    	for(int j = 0; j <= i; j++) {
    		temp_solution.insert(temp_solution.begin() + j, edges[i]);
    		auto cost = cost_tour(network, temp_solution).cost;
    		if (cost < min_cost) {
    			min_cost = cost;
    			constructed_solution = temp_solution;
    		}
    		temp_solution.erase(temp_solution.begin() + j);
    	}
    }

    // Step 3: Update solution
    return cost_tour(network, constructed_solution);
}

// Select 'num_elements' random edges from the source and return the list of chosen edges and remaining edges
std::pair<std::vector<Edge>, std::vector<Edge>> select_random_edges(
    const std::vector<Edge>& source, size_t num_elements) {
    if (num_elements > source.size()) {
        throw std::invalid_argument("num_elements cannot be greater than the source size");
    }

    std::vector<Edge> chosen_edges;
    std::vector<Edge> remaining_edges(source); // Start with all edges in the remaining list

    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    for (size_t i = 0; i < num_elements; ++i) {
        std::uniform_int_distribution<> dis(0, remaining_edges.size() - 1);
        int random_index = dis(gen);

        // Move the chosen edge to the chosen_edges list
        chosen_edges.push_back(remaining_edges[random_index]);
        
        // Remove the chosen edge from the remaining edges
        remaining_edges.erase(remaining_edges.begin() +  random_index);
    }

    return {chosen_edges, remaining_edges};
}

// Takes an edge and find the position of this edge in the list of edges to minimize the cost
Solution simple_shift_operator(const Network& network, std::vector<Edge> edges, const Edge& edge) {
    const int n = edges.size();
    auto min_cost = std::numeric_limits<double>::infinity();
    std::vector<Edge> best_solution = edges;

    for (int i = 0; i <= n; i++) {
        edges.insert(edges.begin() + i, edge);
        auto cost = cost_tour(network, edges).cost;
        if (cost < min_cost) {
            min_cost = cost;
            best_solution = edges;
        }
        edges.erase(edges.begin() + i);
    }
    return cost_tour(network, best_solution);
}

// Algorithm (2)
Solution multiple_shift_operator(const Network& network, Solution solution, double shift_percent=0.5, int max_counter=100, int seed=1234) {
    int counter = 0;
    int number_of_shift_edges = ceil(solution.edges.size() * shift_percent);
    while (counter < max_counter) {
        auto [chosen_edges, remaining_edges] = select_random_edges(solution.edges, number_of_shift_edges);
        for(Edge edge : chosen_edges) {
            remaining_edges = simple_shift_operator(network, remaining_edges, edge).edges;
        }
        auto new_solution = cost_tour(network, remaining_edges);
        if (new_solution.cost < solution.cost) {
            solution = new_solution;
            counter = 0;
        }
        else {
            counter++;
        }
    }
    return solution;
}

// Algorithm (3)
Solution reorder_edges(Network network, Solution solution) {
    std::vector<Edge> perm;
    Vertex cur = network.depot;
    std::map<Edge, bool> moved;
    std::map<Edge, bool> delivery_edges;
    for (int i = 0; i < solution.edges.size(); i++) {
    	// checking the edge solution.edges[i]
        if (delivery_edges.count(solution.edges[i]) > 0) continue;
        Vertex next = solution.direction[i]? solution.edges[i].v : solution.edges[i].u;
        for (auto edge : network.shortest_paths[{cur.id, next.id}]) {
            if (edge.load > 0.0 || (edge.u == edge.v && edge.u == network.depot)) {
                // This is a delivery edge
                if (delivery_edges.count(edge) == 0) {
                    delivery_edges[edge] = true;
                    perm.push_back(edge);
                }
            }
        }
    	if (!perm.empty() && perm.back() == solution.edges[i]) {
			// If we travel to this edge
    		std::cout << "A special case!" << std::endl;
    		cur = solution.direction[i]? perm.back().v : perm.back().u;
    	}
    	else {
    		perm.push_back(solution.edges[i]);
    		delivery_edges[perm.back()] = true;
    		cur = solution.direction[i]? perm.back().u : perm.back().v;
    	}
        if (perm.size() == solution.edges.size()) break;
    }
    my_assert(perm.size() == solution.edges.size(), "Permutation and solution size must be the same");
    auto new_sol =  cost_tour(network, perm);
	my_assert(new_sol.cost <= solution.cost + epsilon, "There is an error in the reorder function!");
    return new_sol;
}

// Greedy Constructive Heuristics Implementation
Solution new_greedy_constructive_heuristics(Network network) {
    // Lambda comparator for edges
    auto edge_comparator = [](const Edge& a, const Edge& b) {
        double productA = a.length * a.load;
        double productB = b.length * b.load;
        return productA > productB; // Larger products come first
    };

    // Multiset of edges with custom comparator
    std::multiset<Edge, decltype(edge_comparator)> edge_pool(edge_comparator);
	for (const auto& edge : network.deliver_edges) {
		edge_pool.insert(edge);
	}

    // Construct solution iteratively
    std::vector<Edge> constructed_solution;

    while (!edge_pool.empty()) {
        // Extract and remove the best edge from the pool
        Edge current_edge = *edge_pool.begin();
        edge_pool.erase(edge_pool.begin());

        // Apply the simple shift operator
        auto temp_solution = simple_shift_operator(network, constructed_solution, current_edge);
    	constructed_solution = temp_solution.edges;

        // Locate the position of the current edge in the temp solution
        int k_prime = -1;
    	for (int i = 0; i < (int) temp_solution.edges.size(); i++) {
    		if (temp_solution.edges[i] == current_edge) {k_prime = i; break;}
    	}
        my_assert(k_prime >= 0, "k_prime must be >= 0");

    	// Update the path from k_prime - 1 to k_prime
    	Vertex u = network.depot;
    	if (k_prime != 0) {
    		my_assert(k_prime > 0, "k_prime must be > 0");
    		u = temp_solution.direction[k_prime - 1]
					   ? temp_solution.edges[k_prime - 1].u
					   : temp_solution.edges[k_prime - 1].v;
    	}
        Vertex v = temp_solution.direction[k_prime]
                   ? temp_solution.edges[k_prime].v
                   : temp_solution.edges[k_prime].u;

        int insert_position = k_prime;
        for (const auto& edge : network.shortest_paths[{u.id, v.id}]) {
	        auto it = edge_pool.end();
        	for (it = edge_pool.begin(); it != edge_pool.end(); ++it) {
        		if ((*it) == edge) {break;}
        	}
            if (it != edge_pool.end()) {
                constructed_solution.insert(constructed_solution.begin() + insert_position, *it);
                edge_pool.erase(it);
                insert_position++;
            }
        	else {
        		int index = -1;
        		for (int i = insert_position + 1; i < (int) constructed_solution.size(); i++) {
					if (constructed_solution[i] == edge) {index = i; break;}
        		}
        		if (index == -1) {continue;}
        		constructed_solution.erase(constructed_solution.begin() + index);
				constructed_solution.insert(constructed_solution.begin() + insert_position, edge);
        		insert_position++;
        	}
        }

    	if (k_prime != (int) temp_solution.edges.size() - 1) {
			// Update the tour from the shortest path between k_prime and k_prime + 1
    		u = temp_solution.direction[k_prime]
					   ? temp_solution.edges[k_prime].u
					   : temp_solution.edges[k_prime].v;
    		v = temp_solution.direction[k_prime + 1]
				   ? temp_solution.edges[k_prime + 1].v
				   : temp_solution.edges[k_prime + 1].u;
    		my_assert(current_edge == constructed_solution[insert_position], "Insert position wrong!");
    		insert_position = insert_position + 1;
    		for (const auto& edge : network.shortest_paths[{u.id, v.id}]) {
    			auto it = edge_pool.end();
    			for (it = edge_pool.begin(); it != edge_pool.end(); ++it) {
    				if ((*it) == edge) {break;}
    			}
    			if (it != edge_pool.end()) {
    				constructed_solution.insert(constructed_solution.begin() + insert_position, *it);
    				edge_pool.erase(it);
    				insert_position++;
    			}
    			else {
    				int index = -1;
    				for (int i = insert_position + 1; i < (int) constructed_solution.size(); i++) {
    					if (constructed_solution[i] == edge) {index = i; break;}
    				}
    				if (index == -1) {continue;}
    				constructed_solution.erase(constructed_solution.begin() + index);
    				constructed_solution.insert(constructed_solution.begin() + insert_position, edge);
    				insert_position++;
    			}
    		}
    	}
    }
	my_assert(constructed_solution.size() == network.deliver_edges.size(), "They must have the same size");
	std::set<int> s;
	for (const auto& edge : constructed_solution) {s.insert(edge.id);}
	my_assert(s.size() == network.deliver_edges.size(), "They have the same size");

    // Finalize and return the constructed solution
    auto sol = cost_tour(network, constructed_solution);
	sol = reorder_edges(network, sol);
	return sol;
}

Solution find_best_reversal(Network network, Solution solution) {
	Solution best_solution = solution;
	double best_cost = solution.cost;

	int n = solution.edges.size();

	for (int start = 0; start < n; ++start) {
		for (int end = start + 1; end < n; ++end) {
			// Create a copy of the current solution
			Solution new_solution = solution;

			// Reverse the sub-sequence
			std::reverse(new_solution.edges.begin() + start, new_solution.edges.begin() + end + 1);

			// Recalculate cost
			Solution evaluated_solution = cost_tour(network, new_solution.edges);

			// Check if the new cost is better
			if (evaluated_solution.cost < best_cost) {
				best_cost = evaluated_solution.cost;
				best_solution = evaluated_solution;
			}
		}
	}

	return best_solution;
}

Solution permutation_search(const Network &network) {
	auto edges = network.deliver_edges;
	std::sort(edges.begin(), edges.end());
	double best_cost = std::numeric_limits<double>::max();
	Solution best_solution;

	// Loop through all permutations
	do {
		auto c = cost_tour(network, edges);
		if (c.cost < best_cost) {
			best_cost = c.cost;
			best_solution = c;
		}
	} while (std::next_permutation(edges.begin(), edges.end()));
	return best_solution;
}


#endif
