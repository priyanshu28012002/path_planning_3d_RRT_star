# RRT* Path Planning Library

This library implements a 3D RRT* (Rapidly-exploring Random Tree Star) algorithm for path planning in a bounded environment with obstacles. It allows the user to define start and goal positions, search bounds, and obstacles in the environment, and then compute an optimized, collision-free path.

---

## Features

- 3D space planning (x, y, z)
- RRT* tree construction with rewiring for optimality
- Obstacle avoidance
- Configurable step size, search area, and max iterations

---

## Class: `RRTStar`

Located in: `rrt_star.hpp` / `rrt_star.cpp`

---

## Setup & Initialization

### `RRTStar::RRTStar()`
Constructor: Initializes parameters and random number generator.

### `~RRTStar()`
Destructor: Cleans up resources if necessary.

---

## Start & Goal

### `bool set_start(const geometry_msgs::msg::Pose& start)`
Sets the start position. Clears previous tree and initializes it with this node.

### `bool set_goal(const geometry_msgs::msg::Pose& goal)`
Sets the target position (goal) for the path.

---

## Configuration Functions

### `void set_step_size(double step)`
Sets the maximum extension distance from a node when growing the tree.

### `void set_max_iterations(int iter)`
Sets how many nodes the algorithm can attempt to generate.

### `void set_search_radius(double radius)`
Sets the base radius to search for nearby nodes (used for rewiring).

### `void set_area_bounds(double min_x, double max_x, double min_y, double max_y, double min_z, double max_z)`
Sets the bounding box of the search space.

---

## Obstacles

### `void add_obstacle(const Obstacle& obstacle)`
Adds an obstacle to the world.

### `void clear_obstacles()`
Clears all obstacles from the environment.

### `bool check_collision(double x1, double y1, double z1, double x2, double y2, double z2) const`
Checks if a line segment intersects with any obstacle.

---

## Path Planning

### `std::vector<geometry_msgs::msg::Pose> path_finding()`
Runs the RRT* algorithm and returns the final path as a sequence of poses.

### `void add_one_more_Node()`
Adds a new node to the tree by:
- Sampling a random point
- Finding the nearest node
- Stepping toward the point (using `steer`)
- Adding and rewiring if it's collision-free and near the goal

---

## Tree Operations

### `geometry_msgs::msg::Pose get_random_point()`
Generates a random pose within the defined bounds.

### `std::shared_ptr<Node> find_nearest_node(const geometry_msgs::msg::Pose& point)`
Finds the node in the tree closest to a given point.

### `geometry_msgs::msg::Pose steer(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)`
Moves from one point toward another by up to `step_size_` distance.

### `bool is_collision_free(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to)`
Returns true if the motion between two points does not collide with obstacles or exceed bounds.

### `std::vector<std::shared_ptr<Node>> find_near_nodes(const std::shared_ptr<Node>& new_node)`
Finds existing nodes within a computed radius from the new node (for rewiring).

### `void rewire(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& near_nodes)`
Rewires nearby nodes to connect to `new_node` if that results in a lower cost.

### `double calculate_cost(const std::shared_ptr<Node>& node)`
Calculates the cumulative cost to reach a node from the start.

### `double get_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b)`
Computes Euclidean distance between two poses.

---

## Output

### `std::vector<geometry_msgs::msg::Pose> get_path()`
Traces and returns the optimized path from start to goal by backtracking parent links.

---


