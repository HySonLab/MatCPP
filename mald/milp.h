#ifndef __MILP__
#define __MILP__

#include "ilcplex/cplex.h"
#include "ilcplex/ilocplex.h"
#include "network.h"
#include "utils.h"

const double eps = 1e-2;

struct MILPSolution {
    std::map<Vertex, std::map<Vertex, std::map<int, int> > > x, y;
	Network network;
	double objective{};
	double lower_bound = std::numeric_limits<double>::infinity(); // A lower bound calculated with Branch-and-Bound
};

class ArcRoutingFormulation {
public:
	explicit ArcRoutingFormulation(const Network& network) : network(network) {
		model = IloModel(env);
		auxiliary_constraints = IloConstraintArray(env);
		init_bounds();
		create_model();
	}

	void limit_objective(double value) {
		IloExpr expr(env);
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			for(const auto& edge : network.edges) {
				auto u = edge.u, v = edge.v;
				double coeff = edge.length * network.vehicle_curb_weight;
				expr += coeff * (x[u][v][k] + x[v][u][k]);
			}
		}
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			for(const auto& edge : network.deliver_edges) {
				auto u = edge.u, v = edge.v;
				double coeff = edge.length * (network.vehicle_curb_weight - edge.load / 2);
				expr += coeff * (y[u][v][k] + y[v][u][k]);
				expr += edge.length * t[edge][k];
			}
		}
		for (int k = 0; k < network.deliver_edges.size() - 1; k++) {
			for (const auto& edge : network.edges) {
				expr += edge.length * z[edge][k];
			}
		}
		auto constraint = IloRange(env, -IloInfinity, expr, value);
		model.add(constraint);
		auxiliary_constraints.add(constraint);
	}

	void fix_x(int start, int end, std::map<Vertex, std::map<Vertex, std::map<int, int> > > x_values) {
		my_assert(start >= 0, "start must be >= 0. Start = " + std::to_string(start));
		my_assert(end <= (int) network.edges.size() - 1, "end should be less than network.edges.size(). End = " + std::to_string(end));
		for (const auto& edge : network.edges) {
			for (int k = start; k <= end; k++) {
				auto constraint = IloRange(env, x_values[edge.u][edge.v][k], x[edge.u][edge.v][k], x_values[edge.u][edge.v][k]);
				model.add(constraint);
				auxiliary_constraints.add(constraint);
				constraint = IloRange(env, x_values[edge.v][edge.u][k], x[edge.v][edge.u][k], x_values[edge.v][edge.u][k]);
				model.add(constraint);
				auxiliary_constraints.add(constraint);
			}
		}
	}

	void fix_y(int start, int end, std::map<Vertex, std::map<Vertex, std::map<int, int> > > y_values) {
		my_assert(start >= 0, "start must be >= 0. Start = " + std::to_string(start));
		my_assert(end <= (int) network.edges.size() - 1, "end should be less than network.edges.size(). End = " + std::to_string(end));
		for (const auto& edge : network.deliver_edges) {
			for (int k = start; k <= end; k++) {
				auto constraint = IloRange(env, y_values[edge.u][edge.v][k], y[edge.u][edge.v][k], y_values[edge.u][edge.v][k]);
				model.add(constraint);
				auxiliary_constraints.add(constraint);
				constraint = IloRange(env, y_values[edge.v][edge.u][k], y[edge.v][edge.u][k], y_values[edge.v][edge.u][k]);
				model.add(constraint);
				auxiliary_constraints.add(constraint);
			}
		}
	}

	void clear_auxiliary_constraints() {
		std::cout << "Clearing " << auxiliary_constraints.getSize() <<" auxiliary constraints" << std::endl;
		model.remove(auxiliary_constraints);
		auxiliary_constraints = IloConstraintArray(env);
		std::cout << "Now the size of auxiliary_constraints is " << auxiliary_constraints.getSize() << std::endl;
	}

	MILPSolution solve() {
		IloCplex solver(env);
		solver.extract(model);
        // Set the parameters
        solver.setParam(IloCplex::Param::Preprocessing::Symmetry, 0); // Disable symmetry detection
        solver.setParam(IloCplex::Param::Emphasis::MIP, IloCplex::MIPEmphasisHiddenFeas);           // Emphasize finding feasible solutions
        solver.setParam(IloCplex::Param::TimeLimit, 100);             // Set time limit to 100 seconds
        solver.setParam(IloCplex::Param::MIP::Limits::Solutions, 3);  // Limit to 5 feasible solutions
		// Supress console output from CPLEX solver
		solver.setOut(env.getNullStream());
        solver.setWarning(env.getNullStream());
		auto solvable = solver.solve();
		if (solvable != IloTrue) {
			std::cout << "Cannot find a solution!" << std::endl;
			return {};
		}
		std::cout << "The solver status is: " << solver.getStatus() << std::endl;
		MILPSolution solution;
		solution.network = network;
		solution.objective = solver.getObjValue();
		solution.lower_bound = solver.getBestObjValue();
		for (const auto& edge : network.edges) {
			auto u = edge.u, v = edge.v;
			if (solution.x.count(u) == 0) {solution.x[u] = {};}
			if (solution.x[u].count(v) == 0) {solution.x[u][v] = {};}
			if (solution.x.count(v) == 0) {solution.x[v] = {};}
			if (solution.x[v].count(u) == 0) {solution.x[v][u] = {};}

			for (int k = 0; k < network.deliver_edges.size(); k++) {
				solution.x[u][v][k] = solver.getIntValue(x[u][v][k]);
				solution.x[v][u][k] = solver.getIntValue(x[v][u][k]);
			}
		}
		for (const auto& edge : network.deliver_edges) {
			auto u = edge.u, v = edge.v;
			if (solution.y.count(u) == 0) {solution.y[u] = {};}
			if (solution.y[u].count(v) == 0) {solution.y[u][v] = {};}
			if (solution.y.count(v) == 0) {solution.y[v] = {};}
			if (solution.y[v].count(u) == 0) {solution.y[v][u] = {};}
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				solution.y[u][v][k] = solver.getIntValue(y[u][v][k]);
				solution.y[v][u][k] = solver.getIntValue(y[v][u][k]);
			}
		}
		return solution;
	}

	Network get_network() {
		return network;
	}
private:
	// Model and evironment
	IloEnv env;
	IloModel model;
	IloConstraintArray auxiliary_constraints;

	// Variables
	std::map<Vertex, std::map<Vertex, std::map<int, IloIntVar> > > x, y;
	std::map<Edge, std::map<int, IloNumVar> > t, z;
	std::map<int, IloNumVar> f;

	// Values
	std::map<int, double> lower_bound_f, upper_bound_f;

	// Network object
	Network network;

	void init_bounds() {
		// Initialize lower_bound_f and upper_bound_f given the information in network
		std::vector<double> loads;
		for(const auto& edge : network.deliver_edges) {
			loads.push_back(edge.load);
		}
		std::sort(loads.begin(), loads.end());
		lower_bound_f[0] = upper_bound_f[0] = network.total_load;
		for (int i=1; i<network.deliver_edges.size(); i++) {
			lower_bound_f[i] = lower_bound_f[i - 1] - loads[network.deliver_edges.size() - i];
			upper_bound_f[i] = upper_bound_f[i - 1] - loads[i - 1];
		}
		for (int i=0; i<network.deliver_edges.size(); i++) {
			my_assert(lower_bound_f[i] <= upper_bound_f[i], "lower_bound_f[i] > upper_bound_f[i]");
		}
	}

	void create_model() {
		create_variables();
		create_objective();
		constraint1();
		constraint2();
		constraint3();
		constraint4();
		constraint5();
		constraint6();
		constraint7();
		constraint8();
		constraint9();
		constraint10();
		constraint11();
	}

	void create_variables() {
		// The variables are defined at https://riunet.upv.es/bitstream/handle/10251/105283/CPP-LC_25042017.pdf?sequence=3
		// x_{vertex_1}{vertex_2}{k}
		// y_{vertex_1}{vertex_2}{k}
		// z_{edge}{k} 
		// t_{edge}{k}
		// f_{k}

		// Helper lambda to ensure a key exists in a nested map structure
	    auto ensure_nested_key = [](auto& map, const auto& key1, const auto& key2) {
	        if (map.find(key1) == map.end()) {
	            map[key1] = {};
	        }
	        if (map[key1].find(key2) == map[key1].end()) {
	            map[key1][key2] = {};
	        }
	    };

		for (const auto& edge : network.deliver_edges) {
			auto u = edge.u, v = edge.v;
	        ensure_nested_key(y, u, v);
	        ensure_nested_key(y, v, u);
			if (t.find(edge) == t.end()) {
				t[edge] = std::map<int, IloNumVar>();
			}
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				y[u][v][k] = IloIntVar(env, 0, 1);
				y[v][u][k] = IloIntVar(env, 0, 1);
				t[edge][k] = IloNumVar(env, 0.0, IloInfinity, ILOFLOAT);
			}
		}

		for (const auto& edge : network.edges) {
			auto u = edge.u, v = edge.v;
	        ensure_nested_key(x, u, v);
	        ensure_nested_key(x, v, u);
			if (z.find(edge) == z.end()) {
				z[edge] = std::map<int, IloNumVar>();
			}
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				x[u][v][k] = IloIntVar(env, 0, 1);
				x[v][u][k] = IloIntVar(env, 0, 1);
				z[edge][k] = IloNumVar(env, 0.0, IloInfinity, ILOFLOAT);
			}
		}

		f[0] = IloNumVar(env, network.total_load, network.total_load, ILOFLOAT);
		f[network.deliver_edges.size()] = IloNumVar(env, 0.0, 0.0, ILOFLOAT);

		for(int k = 1; k < network.deliver_edges.size(); k++) {
			f[k] = IloNumVar(env, lower_bound_f[k], upper_bound_f[k], ILOFLOAT);
		}
	}

	void create_objective() {
		IloExpr expr(env);
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			for(const auto& edge : network.edges) {
				auto u = edge.u, v = edge.v;
				double coeff = edge.length * network.vehicle_curb_weight;
				expr += coeff * (x[u][v][k] + x[v][u][k]);
			}
		}
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			for(const auto& edge : network.deliver_edges) {
				auto u = edge.u, v = edge.v;
				double coeff = edge.length * (network.vehicle_curb_weight - edge.load / 2);
				expr += coeff * (y[u][v][k] + y[v][u][k]);
				expr += edge.length * t[edge][k];
			}
		}
		for (int k = 0; k < network.deliver_edges.size() - 1; k++) {
			for (const auto& edge : network.edges) {
				expr += edge.length * z[edge][k];
			}
		}
		model.add(IloMinimize(env, expr));
	}

	void constraint1() {
		// Every edge is served exactly once and in one direction among the K periods
		for (const auto& edge : network.deliver_edges) {
			auto u = edge.u, v = edge.v;
			IloExpr expr(env);
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				expr += y[u][v][k] + y[v][u][k];
			}
			model.add(IloRange(env, 1, expr, 1));
		}
	}

	void constraint2() {
		// For every period, there is only one edge being served and only in one direction
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			IloExpr expr(env);
			for (const auto& edge : network.deliver_edges) {
				auto u = edge.u, v = edge.v;
				expr += y[u][v][k] + y[v][u][k];
			}
			model.add(IloRange(env, 1, expr, 1));
		}
	}

	void constraint3() {
		// The load at the start of each period equals the load at the previous period
		// subtracting the amount served during the previous period
		for (int k = 0; k < network.deliver_edges.size() - 1; k++) {
			IloExpr expr(env);
			expr += f[k + 1] - f[k];
			for (const auto& edge : network.deliver_edges) {
				auto u = edge.u, v = edge.v;
				expr += edge.load * (y[u][v][k] + y[v][u][k]);
			}
			model.add(IloRange(env, 0.0, expr, 0.0));
		}
	}

	void constraint4() {
		// During the first period, in-degree equals out-degree for every vertex except the depot
		for (int i = 1; i < network.vertices.size(); i++) {
			IloExpr expr(env);
			auto v = network.vertices[i];
			for (const auto& edge : network.edges) {
				if (edge.v == v) {
					expr += x[edge.v][edge.u][0];
					expr -= x[edge.u][edge.v][0];
				}
				else if (edge.u == v) {
					expr -= x[edge.v][edge.u][0];
					expr += x[edge.u][edge.v][0];
				}
			}
			for (const auto& edge : network.deliver_edges) {
				if (edge.v == v) {
					expr += y[edge.v][edge.u][1];
					expr -= y[edge.u][edge.v][0];
				}
				else if (edge.u == v) {
					expr -= y[edge.v][edge.u][0];
					expr += y[edge.u][edge.v][1];
				}
			}
			model.add(IloRange(env, 0, expr, 0));
		}
	}

	void constraint5() {
		// For every period after the first one, and for every vertices, in-degree equals out-degree
		for (int k = 1; k < network.deliver_edges.size() - 1; k++) {
			for (int i = 0; i < network.vertices.size(); i++) {
				IloExpr expr(env);
				auto v = network.vertices[i];
				for (const auto& edge : network.edges) {
					if (edge.v == v) {
						expr += x[edge.v][edge.u][k];
						expr -= x[edge.u][edge.v][k];
					}
					else if (edge.u == v) {
						expr -= x[edge.v][edge.u][k];
						expr += x[edge.u][edge.v][k];	
					}
				}
				for (const auto& edge : network.deliver_edges) {
					if (edge.v == v) {
						expr += y[edge.v][edge.u][k + 1];
						expr -= y[edge.u][edge.v][k];
					}
					else if (edge.u == v) {
						expr -= y[edge.v][edge.u][k];
						expr += y[edge.u][edge.v][k + 1];
					}
				}
				model.add(IloRange(env, 0, expr, 0));
			}
		}
	}

	void constraint6() {
		// The same equality constraints for the last period
		int k = network.deliver_edges.size() - 1;
		for (int i = 1; i < network.vertices.size(); i++) {
			IloExpr expr(env);
			auto vertex = network.vertices[i];
			for (const auto& edge : network.edges) {
				auto u = edge.u, v = edge.v;
				if (vertex == u) {
					expr += x[u][v][k];
					expr -= x[v][u][k];
				}
				else if (vertex == v) {
					expr -= x[u][v][k];
					expr += x[v][u][k];
				}
			}
			for (const auto& edge : network.deliver_edges) {
				auto u = edge.u, v = edge.v;
				if (vertex == u) {
					expr -= y[v][u][k];
				}
				else if (vertex == v) {
					expr -= y[u][v][k];
				}
			}
			model.add(IloRange(env, 0, expr, 0));
		}

		// Constraints at the depot
		IloExpr expr1(env);
		bool expr1_empty = true; // Check if there are any delivery edge out from the depot

		IloExpr expr2(env);
		for (auto edge : network.edges) {
			auto u = edge.u, v = edge.v;
			if (network.depot == u) {
				expr2 += x[v][u][k];
			}
			else if (network.depot == v) {
				expr2 += x[u][v][k];
			}
		}
		for (const auto& edge : network.deliver_edges) {
			auto u = edge.u, v = edge.v;
			if (network.depot == u) {
				expr1_empty = false;
				expr1 += y[u][v][0];
				expr2 += y[v][u][k];
			}
			else if (network.depot == v) {
				expr1_empty = false;
				expr1 += y[v][u][0];
				expr2 += y[u][v][k];
			}
		}
		// This constraint is just a restriction so that we keep the same MILP formulation from the original paper
		my_assert(!expr1_empty, "There should be at least one delivery edge coming out from the depot.");
		model.add(IloRange(env, 1, expr1, 1));
		model.add(IloRange(env, 1, expr2, 1));
	}

	void constraint7() {
		// An edge being deadheaded if only it is served in a previous period
		// or served in this period but opposite direction
		for (const auto& edge : network.deliver_edges) {
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				auto u = edge.u, v = edge.v;

				// One direction
				IloExpr expr(env);
				expr += y[u][v][k];
				expr -= x[v][u][k];
				for (int l = 0; l < k; l++) {
					expr += y[u][v][l] + y[v][u][l];
				}
				model.add(IloRange(env, 0, expr, IloInfinity));

				// The other direction
				IloExpr expr1(env);
				expr1 += y[v][u][k];
				expr1 -= x[u][v][k];
				for (int l = 0; l < k; l++) {
					expr1 += y[u][v][l] + y[v][u][l];
				}
				model.add(IloRange(env, 0, expr1, IloInfinity));
 			}
		}
	}

	void constraint8() {
		// For every period, each edge is deadheaded in one direction only.
		// This is not needed but adding it will help with the computation
		for (const auto& edge : network.edges) {
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				auto u = edge.u, v = edge.v;
				IloExpr expr(env);
				expr += x[u][v][k] + x[v][u][k];
				model.add(IloRange(env, -IloInfinity, expr, 1));
			}
		}
	}

	void constraint9() {
		// This is the constraint (13) in the Corberan 2018 paper
		for (const auto& edge : network.deliver_edges) {
			for (int k = 0; k < network.deliver_edges.size(); k++) {
				auto u = edge.u, v = edge.v;
				IloExpr expr(env);
				IloExpr expr1(env);
				expr += t[edge][k] - lower_bound_f[k] * (y[u][v][k] + y[v][u][k]);
				expr1 += t[edge][k] - upper_bound_f[k] * (y[u][v][k] + y[v][u][k]);
				model.add(IloRange(env, 0, expr, IloInfinity));
				model.add(IloRange(env, -IloInfinity, expr1, 0));
				expr -= f[k];
				expr1 -= f[k];
				model.add(IloRange(env, -IloInfinity, expr, -lower_bound_f[k]));
				model.add(IloRange(env, -upper_bound_f[k], expr1, IloInfinity));
			}
		}
	}

	void constraint10() {
		// This is the constraint (14) in the Corberan 2018 paper
		for (const auto& edge : network.edges) {
			for (int k = 0; k < network.deliver_edges.size() - 1; k++) {
				auto u = edge.u, v = edge.v;
				IloExpr expr(env);
				IloExpr expr1(env);
				expr += z[edge][k] - lower_bound_f[k + 1] * (x[u][v][k] + x[v][u][k]);
				expr1 += z[edge][k] - upper_bound_f[k + 1] * (x[u][v][k] + x[v][u][k]);
				model.add(IloRange(env, 0, expr, IloInfinity));
				model.add(IloRange(env, -IloInfinity, expr1, 0));
				expr -= f[k + 1];
				expr1 -= f[k + 1];
				model.add(IloRange(env, -IloInfinity, expr, -lower_bound_f[k + 1]));
				model.add(IloRange(env, -upper_bound_f[k + 1], expr1, IloInfinity));
			}
		}
	}

	void constraint11() {
		// Constraint (15) in the orginal paper
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			IloExpr expr(env);
			for (const auto& edge : network.deliver_edges) {
				expr += t[edge][k];
			}
			expr -= f[k];
			model.add(IloRange(env, 0, expr, 0));
		}
	}

	void constraint12() {
		// For every period, there is no cycle when deadheading
		for (int k = 0; k < network.deliver_edges.size(); k++) {
			for (int i = 0; i < network.vertices.size(); i++) {
				IloExpr expr(env);
				auto v = network.vertices[i];
				for (const auto& edge : network.edges) {
					if (edge.v == v || edge.u == v) {
						expr += x[edge.v][edge.u][k];
						expr += x[edge.u][edge.v][k];
					}
				}
				model.add(IloRange(env, -IloInfinity, expr, 2));
			}
		}
	}
};

#endif