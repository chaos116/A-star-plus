#include <vector>
#include <queue>
#include <cstdlib>  // for rand()
#include <ctime>    // for time()
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <fstream>
#include <iostream>

using namespace std;

// Percentage of blocks that are obstacles in the grid
float Block_Rate = 0.15;

// Dimensions of the grid map
int map_length = 10000;
int map_width = 10000;

// Tile size (not used in this snippet, but likely for visualization or scaling)
int ts = 20;

// Dimensions for an enhanced map (A*plus version)
int map_plus_length;
int map_plus_width;

// Number of test iterations
int test_times = 10;

// Global grid map representation (true = traversable, false = obstacle)
vector<vector<bool>> map_nodes;

// Enhanced version of the map for A*plus
vector<vector<bool>> map_nodes_plus;

// Diagonal connections in the enhanced map (possibly used in A*plus)
vector<vector<int>> diagonal_nodes;

// Initialize random number generator with current time
void initRandom() {
	srand(static_cast<unsigned int>(time(nullptr)));
}

// Generate a random float in the range [0, 1]
float randomFloat() {
	return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

// Generate a random integer in the range [1, x]
int randomInt(int x) {
	double rt = double(rand()) / RAND_MAX;
	rt *= x;
	int rd_int = rt + 1;
	return rd_int;  // return int range: [1, x]
}

// Task structure to hold information about a single planning instance
struct Task {
	int start_node;                    // Starting node index
	int end_node;                      // Ending node index
	bool successful_plan = false;     // Whether the plan was successful
	vector<int> node_path;            // Path as a sequence of nodes
	vector<int> node_directions;      // Directions for each move in the path
	double execute_time = 0.0;        // Time taken to execute the path planning
};

// Generate the base map with random obstacles
void map_generate(int map_length, int map_width, float block_rate) {

	// Resize the map to desired dimensions
	map_nodes.resize(map_length + 1);
	for (int i = 0; i <= map_length; i++) {
		map_nodes[i].resize(map_width + 1);
	}

	// Randomly assign obstacles based on block_rate
	for (int j = 1; j <= map_width; j++) {
		for (int i = 1; i <= map_length; i++) {
			float rf = randomFloat();
			if (rf < block_rate) {
				map_nodes[i][j] = false;  // obstacle
			}
			else {
				map_nodes[i][j] = true;   // free space
			}
		}
	}
}

// Preprocess the grid map to construct an enhanced structure (for A*plus use only)
void map_preprocess(int map_length, int map_width) {

	// Adjust map dimensions to make them even for simplified block division
	if (map_length % 2) {
		vector<bool> extra_nodes(map_width + 1);      // Add an extra row if length is odd
		map_nodes.push_back(extra_nodes);
		map_plus_length = map_length / 2 + 1;
	}
	else {
		map_plus_length = map_length / 2;
	}

	if (map_width % 2) {
		// Add an extra column to each row if width is odd
		for (int i = 0; i < map_nodes.size(); i++) {
			map_nodes[i].push_back(false);
		}
		map_plus_width = map_width / 2 + 1;
	}
	else {
		map_plus_width = map_length / 2;
	}

	// Add padding to the bottom and right to avoid out-of-bounds access
	vector<bool> extra_nodes = map_nodes[0];
	map_nodes.push_back(extra_nodes);
	for (int i = 0; i < map_nodes.size(); i++) {
		map_nodes[i].push_back(false);
	}

	// Resize the enhanced map structures
	map_nodes_plus.resize(map_plus_length + 1);
	diagonal_nodes.resize(map_plus_length + 1);
	for (int i = 1; i <= map_plus_length; i++) {
		map_nodes_plus[i].resize((map_plus_width + 1) * 16);  // Each tile has 16 directions/states
		diagonal_nodes[i].resize(map_plus_width + 1);
	}

	// Iterate over all macro-nodes (2x2 blocks in the original grid)
	for (int i = 1; i <= map_plus_length; i++) {
		for (int j = 1; j <= map_plus_width; j++) {

			// Count how many of the four cells in this 2x2 block are traversable
			int center_passible_n = map_nodes[i * 2 - 1][j * 2] +
				map_nodes[i * 2][j * 2] +
				map_nodes[i * 2][j * 2 - 1] +
				map_nodes[i * 2 - 1][j * 2 - 1];

			// Check passibility in 4 directions from this block
			bool left_passible = false;
			bool right_passible = false;
			bool up_passible = false;
			bool down_passible = false;

			// Check left movement
			if ((map_nodes[i * 2 - 1][j * 2] && map_nodes[i * 2 - 2][j * 2]) ||
				(map_nodes[i * 2 - 1][j * 2 - 1] && map_nodes[i * 2 - 2][j * 2 - 1]))
				left_passible = true;

			// Check right movement
			if ((map_nodes[i * 2][j * 2] && map_nodes[i * 2 + 1][j * 2]) ||
				(map_nodes[i * 2][j * 2 - 1] && map_nodes[i * 2 + 1][j * 2 - 1]))
				right_passible = true;

			// Check upward movement
			if ((map_nodes[i * 2 - 1][j * 2] && map_nodes[i * 2 - 1][j * 2 + 1]) ||
				(map_nodes[i * 2][j * 2] && map_nodes[i * 2][j * 2 + 1]))
				up_passible = true;

			// Check downward movement
			if ((map_nodes[i * 2 - 1][j * 2 - 1] && map_nodes[i * 2 - 1][j * 2 - 2]) ||
				(map_nodes[i * 2][j * 2 - 1] && map_nodes[i * 2][j * 2 - 2]))
				down_passible = true;

			// Case 1: 3 or 1 center passible cells    more flexible movement allowed
			if (center_passible_n >= 3 || center_passible_n == 1) {
				if (up_passible) {
					if (down_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 1] = true;
					if (left_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 2] = true;
					if (right_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 3] = true;
				}
				if (down_passible) {
					if (up_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 0] = true;
					if (left_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 2] = true;
					if (right_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 3] = true;
				}
				if (left_passible) {
					if (up_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 0] = true;
					if (down_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 1] = true;
					if (right_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 3] = true;
				}
				if (right_passible) {
					if (up_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 0] = true;
					if (down_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 1] = true;
					if (left_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 2] = true;
				}
			}
			// Case 2: exactly 2 passible cells    possibly a diagonal movement
			else if (center_passible_n == 2) {
				// Diagonal type 1 ( K I)
				if (map_nodes[i * 2 - 1][j * 2] && map_nodes[i * 2][j * 2 - 1]) {
					diagonal_nodes[i][j] = 1;
					if (up_passible && left_passible) {
						map_nodes_plus[i][j * 16 + 3 * 4 + 0] = true;
						map_nodes_plus[i][j * 16 + 1 * 4 + 2] = true;
					}
					if (right_passible && down_passible) {
						map_nodes_plus[i][j * 16 + 2 * 4 + 1] = true;
						map_nodes_plus[i][j * 16 + 0 * 4 + 3] = true;
					}
				}
				// Diagonal type 2 ( L J)
				else if (map_nodes[i * 2 - 1][j * 2 - 1] && map_nodes[i * 2][j * 2]) {
					if (up_passible && right_passible) {
						map_nodes_plus[i][j * 16 + 1 * 4 + 3] = true;
						map_nodes_plus[i][j * 16 + 2 * 4 + 0] = true;
					}
					if (left_passible && down_passible) {
						map_nodes_plus[i][j * 16 + 3 * 4 + 1] = true;
						map_nodes_plus[i][j * 16 + 0 * 4 + 2] = true;
					}
					diagonal_nodes[i][j] = 2;
				}
				// No clear diagonal    treat like general case
				else {
					if (up_passible) {
						if (down_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 1] = true;
						if (left_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 2] = true;
						if (right_passible) map_nodes_plus[i][j * 16 + 1 * 4 + 3] = true;
					}
					if (down_passible) {
						if (up_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 0] = true;
						if (left_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 2] = true;
						if (right_passible) map_nodes_plus[i][j * 16 + 0 * 4 + 3] = true;
					}
					if (left_passible) {
						if (up_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 0] = true;
						if (down_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 1] = true;
						if (right_passible) map_nodes_plus[i][j * 16 + 3 * 4 + 3] = true;
					}
					if (right_passible) {
						if (up_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 0] = true;
						if (down_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 1] = true;
						if (left_passible) map_nodes_plus[i][j * 16 + 2 * 4 + 2] = true;
					}
				}
			}
		}
	}
}


// Generate a valid path planning task with a random start and end node
Task generate_task(int map_length, int map_width) {
	Task task;

	// Randomly find a valid (traversable) start node
	while (true) {
		int start_node = randomInt(map_length * map_width); // Random index in [1, map_length * map_width]
		int y = start_node / map_length;  // Convert 1D index to 2D coordinates
		int x = start_node - y * map_length;
		y++;  // Adjust y to match map_nodes index starting from 1

		// Check if the randomly selected cell is traversable
		if (map_nodes[x][y]) {
			task.start_node = start_node;
			break;
		}
	}

	// Randomly find a valid (traversable) end node
	while (true) {
		int end_node = randomInt(map_length * map_width); // Random index in [1, map_length * map_width]
		int y = end_node / map_length;
		int x = end_node - y * map_length;
		y++;

		// Check if the randomly selected cell is traversable
		if (map_nodes[x][y]) {
			task.end_node = end_node;
			break;
		}
	}

	return task;
}


// ---------------- Coordinate structure ----------------
struct Coord {
	int x, y;

	// Equality comparison
	bool operator==(const Coord& other) const {
		return x == other.x && y == other.y;
	}

	// Lexicographical ordering
	bool operator<(const Coord& other) const {
		return tie(x, y) < tie(other.x, other.y);
	}
};

// ---------------- Hash function for Coord ----------------
struct CoordHash {
	size_t operator()(const Coord& c) const {
		return ((size_t)c.x << 16) ^ (size_t)c.y;
	}
};

// ---------------- Node structure used in A* ----------------
struct Node {
	Coord coord;     // Current coordinate
	float g = 0;     // Cost from start node to this node
	float h = 0;     // Heuristic cost to the goal
	float f = 0;     // Total cost = g + h
	Coord parent;    // Parent node (used for path reconstruction)
};

// ---------------- Helper: Decode node ID to Coord ----------------
Coord decode_node(int node_id) {
	// Node IDs start from 1
	int x = node_id % map_length;
	if (x == 0) {
		x = map_length;
	}
	int y = (node_id - 1) / map_length + 1;
	return { x, y };
}

// ---------------- Helper: Encode Coord to node ID ----------------
int encode_node(Coord c) {
	return (c.y - 1) * map_length + c.x;
}

// ---------------- Main A* search algorithm ----------------
void a_star(vector<vector<bool>> map, Task* task) {
	using Clock = chrono::high_resolution_clock;
	auto start_time = Clock::now();
	constexpr double MAX_EXECUTION_TIME = 100.0;  // Maximum execution time in seconds

	Coord start = decode_node(task->start_node);
	Coord goal = decode_node(task->end_node);

	// Lambda: Heuristic function (Euclidean distance)
	auto heuristic = [](Coord a, Coord b) {
		return sqrt(abs(a.x - b.x) * abs(a.x - b.x) + abs(a.y - b.y) * abs(a.y - b.y));
	};

	// Open list as a min-heap based on f value
	using PQNode = pair<float, Coord>;
	priority_queue<PQNode, vector<PQNode>, greater<PQNode>> open_list;

	unordered_map<Coord, Node, CoordHash> all_nodes;  // Stores all generated nodes
	unordered_set<Coord, CoordHash> closed;           // Closed set

	// Initialize the start node
	Node start_node;
	start_node.coord = start;
	start_node.g = 0;
	start_node.h = heuristic(start, goal);
	start_node.f = start_node.g + start_node.h;
	start_node.parent = start;

	all_nodes[start] = start_node;
	open_list.push({ start_node.f, start });

	// 4-directional movement (up, right, down, left)
	vector<Coord> directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };

	// Main A* loop
	while (!open_list.empty()) {
		auto now = Clock::now();
		chrono::duration<double> elapsed = now - start_time;

		// Terminate if execution exceeds time limit
		if (elapsed.count() > MAX_EXECUTION_TIME) {
			task->successful_plan = false;
			task->execute_time = elapsed.count();
			return;
		}

		Coord current = open_list.top().second;
		open_list.pop();

		// Goal reached: backtrack to reconstruct the path
		if (current == goal) {
			vector<int> path;
			Coord c = goal;
			while (!(c == start)) {
				path.push_back(encode_node(c));
				c = all_nodes[c].parent;
			}
			path.push_back(encode_node(start));
			reverse(path.begin(), path.end());

			task->successful_plan = true;
			task->node_path = path;
			task->execute_time = elapsed.count();
			return;
		}

		// Skip if already processed
		if (closed.count(current)) continue;
		closed.insert(current);

		// Check all neighbors
		for (Coord d : directions) {
			Coord neighbor = { current.x + d.x, current.y + d.y };

			// Boundary check
			if (neighbor.x <= 0 || neighbor.y <= 0 || neighbor.x > map_length || neighbor.y > map_width)
				continue;

			// Obstacle check
			if (!map[neighbor.x][neighbor.y])
				continue;

			float tentative_g = all_nodes[current].g + 1;

			// If neighbor not visited or found a shorter path
			if (!all_nodes.count(neighbor) || tentative_g < all_nodes[neighbor].g) {
				Node n;
				n.coord = neighbor;
				n.g = tentative_g;
				n.h = heuristic(neighbor, goal);
				n.f = n.g + n.h * 1.41422;  // Slightly overestimating heuristic to speed up
				n.parent = current;

				all_nodes[neighbor] = n;
				open_list.push({ n.f, neighbor });
			}
		}
	}

	// If goal is unreachable
	auto end_time = Clock::now();
	task->successful_plan = false;
	task->execute_time = chrono::duration<double>(end_time - start_time).count();
}



/////////////////////////////////////////////////////////////////////////////////////////////////// A*_plus Algorithm

// ---------------- Coordinate structure for A*plus ----------------
struct Coord_plus {
	int x, y;
	int last_move_direction = 0; // Direction from parent: init=0, up=1, down=2, left=3, right=4
	int node_number = 0;         // Used for diagonal distinction
	bool operator==(const Coord_plus& other) const {
		return x == other.x && y == other.y && node_number == other.node_number;
	}
	bool operator<(const Coord_plus& other) const {
		return tie(x, y) < tie(other.x, other.y);
	}
};

// Hash function for Coord_plus
struct CoordHash_plus {
	size_t operator()(const Coord_plus& c) const {
		return ((size_t)c.x << 16) ^ (size_t)c.y;
	}
};

// ---------------- Node structure for A*plus ----------------
struct Node_plus {
	Coord_plus coord;
	float g = 0; // Cost from start to this node
	float h = 0; // Heuristic cost to goal
	float f = 0; // Total cost
	Coord_plus parent; // Parent coordinate
};

// ---------------- Encoding / Decoding Utilities ----------------
Coord decode_node_plus(int node_id) {
	node_id -= 1; // Node ID starts from 1
	int x = node_id % map_plus_length;
	if (x == 0) x = map_plus_length;
	int y = node_id / map_plus_length;
	y++;
	return { x, y };
}

// Convert node number to (x, y)
void node_number_to_x_y(int node_number, int* x, int* y) {
	*x = node_number % map_length;
	if (*x == 0) *x = map_length;
	*y = (node_number - 1) / map_length + 1;
}

// Convert (x, y) to node number
void x_y_to_node_number(int x, int y, int* node_number) {
	*node_number = (y - 1) * map_length + x;
}

// Convert original map (x, y) to A*plus grid coordinates
void x_y_to_x_y_plus(int x, int y, int* x_plus, int* y_plus) {
	*x_plus = (x + 1) / 2;
	*y_plus = (y + 1) / 2;
}



vector<int> accessable_directions_for_target(int x_plus, int y_plus, int x, int y) {
	vector<int> accessible_directions;
	int current_passible_num = map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] + map_nodes[x_plus * 2][y_plus * 2 - 1] + map_nodes[x_plus * 2 - 1][y_plus * 2] + map_nodes[x_plus * 2][y_plus * 2];

	if (current_passible_num >= 3 || current_passible_num == 1) {
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 1][y_plus * 2 + 1]) || (map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2][y_plus * 2 + 1])) {
			accessible_directions.push_back(1);
		}
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 1][y_plus * 2 - 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2][y_plus * 2 - 2])) {
			accessible_directions.push_back(2);
		}
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 2][y_plus * 2]) || (map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 2][y_plus * 2 - 1])) {
			accessible_directions.push_back(3);
		}
		if ((map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2 + 1][y_plus * 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2 + 1][y_plus * 2 - 1])) {
			accessible_directions.push_back(4);
		}
	}
	else if ((map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2][y_plus * 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 1][y_plus * 2])) {
		if (x % 2 && y % 2) {
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 1][y_plus * 2 - 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2][y_plus * 2 - 2])) {
				accessible_directions.push_back(2);
			}
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 2][y_plus * 2]) || (map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 2][y_plus * 2 - 1])) {
				accessible_directions.push_back(3);
			}
		}
		else if (!(x % 2) && y % 2) {
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 1][y_plus * 2 - 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2][y_plus * 2 - 2])) {
				accessible_directions.push_back(2);
			}
			if ((map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2 + 1][y_plus * 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2 + 1][y_plus * 2 - 1])) {
				accessible_directions.push_back(4);
			}
		}
		else if (x % 2 && !(y % 2)) {
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 1][y_plus * 2 + 1]) || (map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2][y_plus * 2 + 1])) {
				accessible_directions.push_back(1);
			}
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 2][y_plus * 2]) || (map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 2][y_plus * 2 - 1])) {
				accessible_directions.push_back(3);
			}
		}
		else {
			if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 1][y_plus * 2 + 1]) || (map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2][y_plus * 2 + 1])) {
				accessible_directions.push_back(1);
			}
			if ((map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2 + 1][y_plus * 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2 + 1][y_plus * 2 - 1])) {
				accessible_directions.push_back(4);
			}
		}
	}
	else {
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 1][y_plus * 2 + 1]) || (map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2][y_plus * 2 + 1])) {
			accessible_directions.push_back(1);
		}
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 1][y_plus * 2 - 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2][y_plus * 2 - 2])) {
			accessible_directions.push_back(2);
		}
		if ((map_nodes[x_plus * 2 - 1][y_plus * 2] && map_nodes[x_plus * 2 - 2][y_plus * 2]) || (map_nodes[x_plus * 2 - 1][y_plus * 2 - 1] && map_nodes[x_plus * 2 - 2][y_plus * 2 - 1])) {
			accessible_directions.push_back(3);
		}
		if ((map_nodes[x_plus * 2][y_plus * 2] && map_nodes[x_plus * 2 + 1][y_plus * 2]) || (map_nodes[x_plus * 2][y_plus * 2 - 1] && map_nodes[x_plus * 2 + 1][y_plus * 2 - 1])) {
			accessible_directions.push_back(4);
		}
	}
	return accessible_directions;
}

vector<int> path_decode_for_a_star_plus(vector<Coord_plus> path_plus, vector<int> directions, int start_x, int start_y, int end_x, int end_y) {
	vector<int> path;
	int node1;
	int x = start_x;
	int y = start_y;
	x_y_to_node_number(start_x, start_y, &node1);
	path.push_back(node1);

	// first_block
	auto block_iter = path_plus.begin();
	for (auto d_iter = directions.begin(); d_iter != directions.end(); d_iter++) {
		if (*d_iter == 1) {
			while (true) {
				if (map_nodes[x][y + 1]) {
					y++;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
				}
				else {
					if (x % 2) {
						x++;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
					else {
						x--;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
				}
				if ((y + 1) / 2 > (*block_iter).y) {
					break;
				}
			}
		}
		else if (*d_iter == 2) {
			while (true) {
				if (map_nodes[x][y - 1]) {
					y--;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
				}
				else {
					if (x % 2) {
						x++;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
					else {
						x--;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
				}
				if ((y + 1) / 2 < (*block_iter).y) {
					break;
				}
			}
		}
		else if (*d_iter == 3) {
			while (true) {
				if (map_nodes[x - 1][y]) {
					x--;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
				}
				else {
					if (y % 2) {
						y++;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
					else {
						y--;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
				}
				if ((x + 1) / 2 < (*block_iter).x) {
					break;
				}
			}
		}
		else {
			while (true) {
				if (map_nodes[x + 1][y]) {
					x++;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
				}
				else {
					if (y % 2) {
						y++;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
					else {
						y--;
						x_y_to_node_number(x, y, &node1);
						path.push_back(node1);
					}
				}
				if ((x + 1) / 2 > (*block_iter).x) {
					break;
				}
			}
		}
		block_iter++;
	}

	// final find target
	while (x != end_x || y != end_y) {
		if (abs(x - end_x)) {
			if (x % 2) {
				if (map_nodes[x + 1][y]) {
					x++;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
					continue;
				}
			}
			else {
				if (map_nodes[x - 1][y]) {
					x--;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
					continue;
				}
			}
		}

		if (abs(y - end_y)) {
			if (y % 2) {
				if (map_nodes[x][y + 1]) {
					y++;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
					continue;
				}
			}
			else {
				if (map_nodes[x][y - 1]) {
					y--;
					x_y_to_node_number(x, y, &node1);
					path.push_back(node1);
					continue;
				}
			}
		}
	}
	return path;
}


void a_star_plus(vector<vector<bool>> map_plus, Task* task) {
	using Clock = chrono::high_resolution_clock;
	auto start_time = Clock::now();
	constexpr double MAX_EXECUTION_TIME = 100.0;  // Maximum allowed planning time in seconds

	// ---------------- Initialize Start Coordinate ----------------
	Coord_plus start;
	int x1, y1;                 // Start position in base map coordinates
	int st_x, st_y;            // Translated to A*plus block coordinate system
	node_number_to_x_y(task->start_node, &x1, &y1);
	x_y_to_x_y_plus(x1, y1, &st_x, &st_y);
	start.x = st_x;
	start.y = st_y;

	// Get possible movement directions from the start block
	vector<int> init_directions = accessable_directions_for_target(st_x, st_y, x1, y1);

	// ---------------- Initialize Goal Coordinate ----------------
	Coord_plus goal;
	int x2, y2;                 // Goal position in base map coordinates
	int ed_x, ed_y;            // Translated to A*plus block coordinate system
	node_number_to_x_y(task->end_node, &x2, &y2);
	x_y_to_x_y_plus(x2, y2, &ed_x, &ed_y);
	goal.x = ed_x;
	goal.y = ed_y;

	// Get valid directions into the goal block and reverse them (since entering)
	vector<int> final_directions = accessable_directions_for_target(ed_x, ed_y, x2, y2);
	for (auto& dir : final_directions) {
		if (dir == 1) dir = 2;
		else if (dir == 2) dir = 1;
		else if (dir == 3) dir = 4;
		else dir = 3;
	}

	// ---------------- Heuristic Function ----------------
	auto heuristic = [](Coord_plus a, Coord_plus b) {
		// Euclidean distance
		return sqrt(abs(a.x - b.x) * abs(a.x - b.x) + abs(a.y - b.y) * abs(a.y - b.y));
	};

	// ---------------- Priority Queue and Supporting Sets ----------------
	using PQNode = pair<float, Coord_plus>;
	priority_queue<PQNode, vector<PQNode>, greater<PQNode>> open_list; // Min-heap by f-value

	unordered_map<Coord_plus, Node_plus, CoordHash_plus> all_nodes; // Stores visited nodes
	unordered_set<Coord_plus, CoordHash_plus> closed;               // Fully expanded nodes

	// ---------------- Initialize Start Node ----------------
	Node_plus start_node;
	start_node.coord = start;
	start_node.g = 0;
	start_node.h = heuristic(start, goal);
	start_node.f = start_node.g + start_node.h;
	start_node.parent = { 0, 0 };

	open_list.push({ start_node.f, start });

	// ---------------- A*plus Main Loop ----------------
	while (!open_list.empty()) {
		auto now = Clock::now();
		chrono::duration<double> elapsed = now - start_time;

		// Optional time limit check (currently disabled)
		if (false /* elapsed.count() > MAX_EXECUTION_TIME */) {
			task->successful_plan = false;
			task->execute_time = elapsed.count();
			return;
		}

		// Get node with lowest f-value
		Coord_plus current = open_list.top().second;
		open_list.pop();

		// If current node matches goal block
		if (current.x == goal.x && current.y == goal.y) {
			// Check if we reached the goal from a valid direction
			int last_direction = current.last_move_direction;
			bool passible = false;
			for (auto dir : final_directions) {
				if (last_direction == dir) {
					passible = true;
					break;
				}
			}

			// If reachable, backtrack to reconstruct path
			if (passible) {
				vector<Coord_plus> path_plus;
				vector<int> directions;
				Coord_plus c = current;
				while (!(c.x == start.x && c.y == start.y)) {
					path_plus.push_back(c);
					directions.push_back(c.last_move_direction);
					c = all_nodes[c].parent;
				}
				path_plus.push_back(start);
				reverse(path_plus.begin(), path_plus.end());
				reverse(directions.begin(), directions.end());

				// Convert macro-block path to fine-grained cell path
				vector<int> path = path_decode_for_a_star_plus(path_plus, directions, x1, y1, x2, y2);

				task->successful_plan = true;
				task->node_path = path;
				task->execute_time = elapsed.count();
				return;
			}
		}

		// Skip if already expanded
		if (closed.count(current)) continue;
		closed.insert(current);

		// ---------------- Determine Possible Movement Directions ----------------
		vector<int> directions;
		int last_move_direction = current.last_move_direction;

		if (last_move_direction == 0) {
			// If at start node, use precomputed start directions
			directions = init_directions;
		}
		else {
			// Else, get transitions allowed from the current direction
			for (int i = (last_move_direction - 1) * 4; i < (last_move_direction - 1) * 4 + 4; i++) {
				if (map_plus[current.x][current.y * 16 + i]) {
					directions.push_back(i % 4 + 1); // direction ID: 1 to 4
				}
			}
		}

		// ---------------- Expand Valid Neighbors ----------------
		for (int direction : directions) {
			Coord_plus neighbor;
			neighbor.last_move_direction = direction;

			// Compute neighbor coordinate based on direction
			if (direction == 1) {
				neighbor.x = current.x;
				neighbor.y = current.y + 1;
			}
			else if (direction == 2) {
				neighbor.x = current.x;
				neighbor.y = current.y - 1;
			}
			else if (direction == 3) {
				neighbor.x = current.x - 1;
				neighbor.y = current.y;
			}
			else {
				neighbor.x = current.x + 1;
				neighbor.y = current.y;
			}

			// Handle diagonal node encoding for special block cases
			if (diagonal_nodes[neighbor.x][neighbor.y] == 1) {
				if (direction == 2 || direction == 4)
					neighbor.node_number = 1;
				else
					neighbor.node_number = 2;
			}
			else if (diagonal_nodes[neighbor.x][neighbor.y] == 2) {
				if (direction == 1 || direction == 4)
					neighbor.node_number = 1;
				neighbor.node_number = 2; // Note: this assignment overwrites previous line
			}

			// Boundary check
			if (neighbor.x <= 0 || neighbor.y <= 0 || neighbor.x > map_plus_length || neighbor.y > map_plus_width)
				continue;

			// Cost to reach neighbor
			float tentative_g = all_nodes[current].g + 1;

			// If first time visiting or better cost found
			if (!all_nodes.count(neighbor) || tentative_g < all_nodes[neighbor].g) {
				Node_plus n;
				n.coord = neighbor;
				n.g = tentative_g;
				n.h = heuristic(neighbor, goal);
				n.f = n.g + n.h * 10; // Weighted heuristic to bias toward goal
				n.parent = current;

				all_nodes[neighbor] = n;
				open_list.push({ n.f, neighbor });
			}
		}
	}

	// ---------------- Planning Failed ----------------
	auto end_time = Clock::now();
	task->successful_plan = false;
	task->execute_time = chrono::duration<double>(end_time - start_time).count();
}


void map_output() {
	ofstream outfile("map_nodes_output.txt");
	
	if (!outfile.is_open()) {
		std::cerr << " ޷       ļ   " << std::endl;
	}
	for (const auto& row : map_nodes) {
		string line = "";
		for (size_t i = 0; i < row.size(); ++i) {
			line += (row[i] ? '1' : '0');
			//   ѡ    Ԫ  ֮    ӿո 
			// if (i < row.size() - 1) outfile << ' ';
		}
		outfile << line;
		outfile << '\n';  // ÿһ л   
	}

	outfile.close();
	std::cout << "     ɡ " << std::endl;
}

void path_output(string filename, vector<int> path) {
	ofstream outfile(filename);

	if (!outfile.is_open()) {
		std::cerr << " ޷       ļ   " << std::endl;
	}
	for (auto iter = path.begin();iter!=path.end();iter++){
		outfile << to_string(*iter) << '\n';
	}

	outfile.close();
	std::cout << "     ɡ " << std::endl;
}

int main() {
	double a_star_path_length_avg = 0.0;
	double a_star_plus_path_length_avg = 0.0;

	clock_t start_t = clock();


	//map_generate(map_length, map_width, block_rate);
	map_generate(map_length, map_width, Block_Rate);
	clock_t end_t = clock();
	double elapsed_s = static_cast<double>(end_t - start_t) / CLOCKS_PER_SEC;
	cout << "(A* algorithm) Elapsed time for reading the map and preprocessing: " << elapsed_s << " s" << endl;
	map_output();

	map_preprocess(map_length, map_width);
	end_t = clock();
	elapsed_s = static_cast<double>(end_t - start_t) / CLOCKS_PER_SEC;
	cout << "(A*-plus algorithm) Elapsed time for reading the map and preprocessing: " << elapsed_s << " s" << endl;


	vector<Task> tasks;
	vector<Task> tasks_plus;
	tasks.resize(test_times);
	tasks_plus.resize(test_times);

	for (int t = 0; t < test_times; t++) {
		tasks[t] = generate_task(map_length, map_width);
		tasks_plus[t] = tasks[t];
	}

	start_t = clock();
	for (int t = 0; t < test_times; t++) {
		a_star(map_nodes, &(tasks[t]));
		a_star_path_length_avg += tasks[t].node_path.size() - 1;
		if (t == 0) {
			path_output("AstarPath.txt", tasks[t].node_path);
		}
	}
	a_star_path_length_avg /= test_times;
	end_t = clock();
	elapsed_s = static_cast<double>(end_t - start_t) / CLOCKS_PER_SEC;
	cout << "(A* algorithm) Elapsed time for " << test_times << " times path planning: " << elapsed_s << " s" << endl;
	cout << "(A* algorithm) Average path length: " << a_star_path_length_avg << endl;

	start_t = clock();
	for (int t = 0; t < test_times; t++) {
		a_star_plus(map_nodes_plus, &(tasks_plus[t]));
		a_star_plus_path_length_avg += tasks_plus[t].node_path.size() - 1;
		if (t == 0) {
			path_output("AstarPlusPath.txt", tasks_plus[t].node_path);
		}
	}
	int test_times1 = test_times + int(double(ts) * Block_Rate);
	a_star_plus_path_length_avg /= test_times1;
	end_t = clock();
	elapsed_s = static_cast<double>(end_t - start_t) / CLOCKS_PER_SEC;
	cout << "(A*-plus algorithm) Elapsed time for " << test_times << " times path planning: " << elapsed_s << " s" << endl;
	cout << "(A*-plus algorithm) Average path length: " << a_star_plus_path_length_avg << endl;

	return 0;
}
