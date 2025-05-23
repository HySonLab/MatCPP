// Graph algorithms and data structure for the load-dependent Chinese postman problem
// Author: Dr. Truong Son Hy
// Copyright 2023

#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <thread>
#include <algorithm>
#include <queue>
#include <assert.h>

using namespace std;

// +----------+
// | Infinity |
// +----------+
const double INF = 1e9;


// +----------------+
// | Edge structure |
// +----------------+
struct Edge {
	// Constructors
	Edge() {
	}

	Edge(const Edge& another) {
		first = another.first;
		second = another.second;
		d = another.d;
		q = another.q;
	}

	Edge(const int first_, const int second_, const double d_, const double q_) {
		first = first_;
		second = second_;
		d = d_;
		q = q_;
	}

	// For sorting decreasingly
	bool operator < (const Edge& another) {
		if (this -> d * this -> q > another.d * another.q) {
			return true;
		}
		return false;
	}

	bool operator == (const Edge& other) const {
        return (first == other.first && second == other.second) || (first == other.second && second == other.first);
    }

	int first;
	int second;
	double d;
	double q;
};


// +----------------------+
// | Coordinate structure |
// +----------------------+
struct Coordinate {
	Coordinate(const double x_, const double y_) {
		x = x_;
		y = y_;
	}

	double x;
	double y;
};


// +-------------------------------------------+
// | Convert a string into a vector of strings |
// +-------------------------------------------+
vector<string> parse_line(const string str, const char separator = ' ') {
	vector<string> result;
	result.clear();
	int i = 0;
	while (i < str.length()) {
		if (str[i] == separator) {
			++i;
			continue;
		}
		string word = "";
		for (int j = i; j < str.length(); ++j) {
			if (str[j] == separator) {
				break;
			} else {
				word += str[j];
			}
		}
		result.push_back(word);
		i += word.length();
	}
	return result;
}


// +------------------------------+
// | Convert a matrix to a string |
// +------------------------------+
string to_str(const vector< vector<double> > matrix) {
    string result = "";
    for (int i = 0; i < matrix.size(); ++i) {
        for (int j = 0; j < matrix[i].size(); ++j) {
            result += to_string(matrix[i][j]) + " ";
        }
        if (i + 1 < matrix.size()) {
            result += "\n";
        }
    }
    return result;
}


// +------------------------------+
// | Convert a vector to a string |
// +------------------------------+
string to_str(const vector<int> vect) {
    string result = "";
    for (int i = 0; i < vect.size(); ++i) {
        result += to_string(vect[i]) + " ";
    }
    return result;
}


// +-----------------+
// | Graph structure |
// +-----------------+
class Graph {
public:
	// Constructor
	Graph(){
	}

	// Filter for the must-deliver edges
	void filter_must_deliver_edges() {
		this -> deliver_edges.clear();
        for (int i = 0; i < this -> edges.size(); ++i) {
            if (this -> edges[i].q > 0.0) {
                this -> deliver_edges.push_back(Edge(this -> edges[i]));
            }
        }
        assert(this -> deliver_edges.size() > 0);
		this -> num_deliver_edges = this -> deliver_edges.size();
	}

	// Constructor given another object
	Graph(Graph *another) {
		this -> num_nodes = another -> num_nodes;
		this -> num_edges = another -> num_edges;
		this -> W = another -> W;

		this -> edges.clear();
		for (int i = 0; i < another -> edges.size(); ++i) {
			this -> edges.push_back(another -> edges[i]);
		}

		this -> position.clear();
		for (int i = 0; i < another -> position.size(); ++i) {
			this -> position.push_back(another -> position[i]);
		}

		// Filter for the must-deliver edges
		this -> filter_must_deliver_edges();
	}

	// Constructor given the content
	Graph(const int num_nodes, const int num_edges, const double W, const vector<Edge> edges, const vector<Coordinate> position) {
        this -> num_nodes = num_nodes;
        this -> num_edges = num_edges;
		this -> W = W;

        this -> edges.clear();
        for (int i = 0; i < edges.size(); ++i) {
            this -> edges.push_back(edges[i]);
        }

        this -> position.clear();
        for (int i = 0; i < position.size(); ++i) {
            this -> position.push_back(position[i]);
        }

		// Filter for the must-deliver edges
        this -> filter_must_deliver_edges();
    }

	// Load the content from file
	void load_from_file(const string file_name) {
		ifstream file;
        file.open(file_name);

        string str;
        getline(file, str);
        getline(file, str);
        this -> num_nodes = stoi(str);

        getline(file, str);
        getline(file, str);
        this -> num_edges = stoi(str);

        getline(file, str);
        getline(file, str);
        this -> W = stof(str);

        getline(file, str);
        this -> edges.clear();
        for (int i = 0; i < this -> num_edges; ++i) {
            getline(file, str);
            vector<string> words = parse_line(str);
            assert(words.size() == 4);
            const int first = stoi(words[0]);
            const int second = stoi(words[1]);
            const double d = stof(words[2]);
            const double q = stof(words[3]);
            this -> edges.push_back(Edge(first, second, d, q));
        }

        getline(file, str);
        this -> position.clear();
        for (int i = 0; i < this -> num_nodes; ++i) {
            getline(file, str);
            vector<string> words = parse_line(str);
            assert(words.size() == 2);
            const double x = stof(words[0]);
            const double y = stof(words[1]);
            this -> position.push_back(Coordinate(x, y));
        }

        file.close();
	}

	// Constructor given a file
	Graph(const string file_name) {
		load_from_file(file_name);

		// Filter for the must-deliver edges
        this -> filter_must_deliver_edges();
	}

	// Convert the content to a string
	string get_content() {
		string result = "";
		result += "Number of nodes:\n";
		result += to_string(this -> num_nodes) + "\n";
		result += "Number of edges:\n";
		result += to_string(this -> num_edges) + "\n";
		result += "W:\n";
		result += to_string(this -> W) + "\n";
		result += "Edges (node i, node j, d_ij, q_ij):\n";
		for (int i = 0; i < this -> edges.size(); ++i) {
			result += to_string(this -> edges[i].first) + " " + to_string(this -> edges[i].second) + " " + to_string(this -> edges[i].d) + " " + to_string(this -> edges[i].q) + "\n";
		}
		result += "Coordinates:\n";
		for (int i = 0; i < this -> position.size(); ++i) {
			result += to_string(this -> position[i].x) + " " + to_string(this -> position[i].y) + "\n";
		}
		return result;
	}

	// Save the object to file
	void save_to_file(const string file_name) {
		ofstream file;
		file.open(file_name);
		file << get_content();
		file.close();
	}

	// Search edge
	Edge get_edge(const int i, const int j) {
		for (int k = 0; k < this -> edges.size(); ++k) {
			const int first = this -> edges[k].first;
			const int second = this -> edges[k].second;
			const double d = this -> edges[k].d;
			const double q = this -> edges[k].q;
			if ((first == i) && (second == j)) {
				return Edge(i, j, d, q);
			}
			if ((first == j) && (second == i)) {
				return Edge(i, j, d, q);
			}
		}
		return Edge(-1, -1, -1, -1);
	}

	// Floyd's algorithm to find all-pair shortest paths
	void Floyd_algorithm() {
		this -> shortest_path.clear();
		for (int i = 0; i < this -> num_nodes; ++i) {
			vector<double> vect;
			vect.clear();
			for (int j = 0; j < this -> num_nodes; ++j) {
				vect.push_back(INF);
			}
			this -> shortest_path.push_back(vect);
		}

		for (int i = 0; i < this -> num_nodes; ++i) {
			this -> shortest_path[i][i] = 0;
		}

		for (int i = 0; i < this -> num_edges; ++i) {
			const int first = this -> edges[i].first;
			const int second = this -> edges[i].second;
			const double d = this -> edges[i].d;

			if (d < this -> shortest_path[first][second]) {
				this -> shortest_path[first][second] = d;
			}

			if (d < this -> shortest_path[second][first]) {
                this -> shortest_path[second][first] = d;
            }
		}

		for (int k = 0; k < this -> num_nodes; ++k) {
			for (int i = 0; i < this -> num_nodes; ++i) {
				for (int j = 0; j < this -> num_nodes; ++j) {
					if (this -> shortest_path[i][j] > this -> shortest_path[i][k] + this -> shortest_path[k][j]) {
						this -> shortest_path[i][j] = this -> shortest_path[i][k] + this -> shortest_path[k][j];
					}
				}
			}
		}
	}

	// Dijkstra's algorithm to find all-pair shortest paths
	void Dijkstra_algorithm() {
		// Adjacency
		vector< vector< pair<int, double> > > adj;

		for (int i = 0; i < this -> num_nodes; ++i) {
			vector< pair<int, double> > vect;
			vect.clear();
			adj.push_back(vect);
		}

		assert(this -> num_edges == this -> edges.size());
		for (int i = 0; i < this -> num_edges; ++i) {
			const int first = this -> edges[i].first;
			const int second = this -> edges[i].second;
			const double d = this -> edges[i].d;
			const double q = this -> edges[i].q;

			adj[first].push_back(make_pair(second, d));
			adj[second].push_back(make_pair(first, d));
		}

		// For all pairs' shortest paths
		this -> dijkstra_d.clear();
		for (int i = 0; i < this -> num_nodes; ++i) {
			vector<double> vect;
			vect.clear();
			for (int j = 0; j < this -> num_nodes; ++j) {
				vect.push_back(INF);
			}
			this -> dijkstra_d.push_back(vect);
			this -> dijkstra_d[i][i] = 0;
		}

		this -> dijkstra_path.clear();
		for (int i = 0; i < this -> num_nodes; ++i) {
			vector< vector<int> > matrix;
			matrix.clear();
			for (int j = 0; j < this -> num_nodes; ++j) {
				vector<int> vect;
				vect.clear();
				matrix.push_back(vect);
			}
			dijkstra_path.push_back(matrix);
		}

		// Dijkstra's algorithm
		for (int start = 0; start < this -> num_nodes; ++start) {
			// Memories
			vector<double> d;
			vector<int> trace;
			priority_queue< pair<double, int> > q;

			// Initialization
			d.clear();
			trace.clear();

			for (int i = 0; i < this -> num_nodes; ++i) {
				d.push_back(INF);
				trace.push_back(-1);
			}

			d[start] = 0;
			q.push(make_pair(0, start));

			// Algorithm
			while (!q.empty()) {
				const int u = q.top().second;
				q.pop();
				for (int i = 0; i < adj[u].size(); ++i) {
					const int v = adj[u][i].first;
					const double w = adj[u][i].second;
					if (d[v] > d[u] + w) {
						d[v] = d[u] + w;
						trace[v] = u;
						q.push(make_pair(-d[v], v));
					}
				}
			}

			// Get the results
			for (int i = 0; i < this -> num_nodes; ++i) {
				// Get the distance
				this -> dijkstra_d[start][i] = d[i];

				// Get the path
				if (d[i] == INF) {
					continue;
				}

				if (i == start) {
					continue;
				}

				vector<int> path;
				path.clear();
				int v = i;
				while (true) {
					path.push_back(v);
					if (v == start) {
						break;
					}
					assert(trace[v] != -1);
					v = trace[v];
				}

				this -> dijkstra_path[start][i].clear();
				for (int j = path.size() - 1; j >= 0; --j) {
					this -> dijkstra_path[start][i].push_back(path[j]);
				}
			}
		}
	}

	// Staring node is always indexed as 0
	const int start_node = 0;

	// Number of nodes
	int num_nodes;

	// Number of edges
	int num_edges;

	// Number of edges that must be delivered
	int num_deliver_edges;

	// W
	double W;

	// Edges
	vector<Edge> edges;

	// Must-deliver edges
	vector<Edge> deliver_edges;

	// Position of each node
	vector<Coordinate> position;

	// All-pair shortest paths from Floyd's algorithm
	vector< vector<double> > shortest_path;

	// All-pair shortest paths from Dijkstra's algorithm
	vector< vector<double> > dijkstra_d;
	vector< vector< vector<int> > > dijkstra_path;
};


// +---------------------+
// | Dynamic Programming |
// +---------------------+
static pair< vector< vector<double> >, vector<int> > dynamic_programming(Graph *graph, const vector<Edge> sequence) {
	// Start node
	const int start_node = graph -> start_node;

	// Check if the graph has computed Floyd's algorithm before
    assert(graph -> shortest_path.size() == graph -> num_nodes);
	const double W = graph -> W;

	// Number of edges in this list
	const int m = sequence.size();

	// Initialize
    vector< vector<double> > f;
    f.clear();
    for (int k = 0; k < m; ++k) {
        vector<double> vect;
        vect.clear();
        vect.push_back(INF);
        vect.push_back(INF);
        f.push_back(vect);
    }

    vector<int> direction;
    direction.clear();
    for (int k = 0; k < m; ++k) {
        direction.push_back(-1);
    }

	// Special case if there is only one edge in the list
	if (m == 1) {
		// Current edge's information
        const int i = sequence[0].first;
        const int j = sequence[0].second;
        const double q = sequence[0].q;
        const double d = sequence[0].d;

		const double option_0 = (W + q / 2) * d + (W + q) * graph -> shortest_path[start_node][i] + W * graph -> shortest_path[j][start_node];
		const double option_1 = (W + q / 2) * d + (W + q) * graph -> shortest_path[start_node][j] + W * graph -> shortest_path[i][start_node];

		if (option_0 <= option_1) {
			f[0][0] = option_0;
			direction[0] = 0;
		} else {
			f[0][0] = option_1;
			direction[0] = 1;
		}

		return make_pair(f, direction);
	}

	// For tracing
	vector< vector<int> > choice;
    choice.clear();
    for (int k = 0; k < m; ++k) {
        vector<int> vect;
        vect.clear();
        vect.push_back(-1);
        vect.push_back(-1);
        choice.push_back(vect);
    }

	// Compute the Q
	vector<double> Q;
	for (int k = 0; k < m; ++k) {
		Q.push_back(0);
	}
	Q[m - 1] = sequence[m - 1].q;
	for (int k = m - 2; k >= 0; --k) {
		Q[k] = Q[k + 1] + sequence[k].q;
	}

	// Dynamic Programming
	for (int k = m - 1; k >= 0; --k) {
		for (int c = 0; c <= 1; ++c) {
			// Skip this case
			if ((k == 0) && (c == 1)) {
				continue;
			}

			// Current edge's information
			const int i = sequence[k].first;
			const int j = sequence[k].second;
			const double q = sequence[k].q;
			const double d = sequence[k].d;

			// Previous node
			int prev = -1;
			if (k == 0) {
				prev = start_node;
			} else {
            	if (c == 0) {
            		// If the previous edge is not flipped
            		prev = sequence[k - 1].second;
				} else {
                	// If the previous edge is flipped
                	prev = sequence[k - 1].first;
            	}
			}

			double option_0;
			double option_1;

			if (k == m - 1) {
				// For the last edge
				option_0 = (W + q / 2) * d + (W + q) * graph -> shortest_path[prev][i] + W * graph -> shortest_path[j][start_node];
				option_1 = (W + q / 2) * d + (W + q) * graph -> shortest_path[prev][j] + W * graph -> shortest_path[i][start_node];
			} else {
				// For all other edges
				option_0 = (W + Q[k] - q / 2) * d + (W + Q[k]) * graph -> shortest_path[prev][i] + f[k + 1][0];
               	option_1 = (W + Q[k] - q / 2) * d + (W + Q[k]) * graph -> shortest_path[prev][j] + f[k + 1][1];
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

	return make_pair(f, direction);
}


// +--------------+
// | Compute cost |
// +--------------+
double compute_cost(Graph *graph, const vector<Edge> sequence, const vector<int> direction) {
	// Check if the graph has computed Floyd's algorithm before
    assert(graph -> shortest_path.size() == graph -> num_nodes);
    const double W = graph -> W;

    // Number of edges in this list
    const int m = sequence.size();

	// Compute Q
	double Q = 0;
	for (int k = 0; k < sequence.size(); ++k) {
		Q += sequence[k].q;
	}

	// Compute cost
	double cost = 0;

	// Start node
	const int start_node = graph -> start_node;

	// Previous node
	int prev = start_node;

	for (int k = 0; k < sequence.size(); ++k) {
		// Current edge's information
		const int i = sequence[k].first;
        const int j = sequence[k].second;
        const double q = sequence[k].q;
        const double d = sequence[k].d;

		// First edge
		if (direction[k] == 0) {
			cost += (W + Q) * graph -> shortest_path[prev][i];
		} else {
			cost += (W + Q) * graph -> shortest_path[prev][j];
		}

		// Last edge
		if (k == m - 1) {
			if (direction[k] == 0) {
                cost += W * graph -> shortest_path[j][start_node];
            } else {
                cost += W * graph -> shortest_path[i][start_node];
            }
		}

		// Deliver cost
		cost += (W + Q - q / 2) * d;

		// Reduce load
		Q -= q;

		// Update the previous node
		if (direction[k] == 0) {
			prev = j;
		} else {
			prev = i;
		}
	}

	return cost;
}

#endif //GRAPH_H
