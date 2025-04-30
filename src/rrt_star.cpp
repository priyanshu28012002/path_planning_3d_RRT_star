#include "path_planning/rrt_star.hpp"
#include <iostream>
#include <random>

namespace rrt_star {

RRTStar::RRTStar() :
    step_size_(0.5),
    max_iterations_(3000),
    search_radius_(1.0),
    min_x_(-10.0), max_x_(10.0),
    min_y_(-10.0), max_y_(10.0),
    min_z_(-10.0), max_z_(10.0),
    goal_reached_(false)
{
    // Initialize random number generator
    std::srand(std::time(nullptr));
}

RRTStar::~RRTStar() {
    // Clean up if needed
}

bool RRTStar::set_goal(const geometry_msgs::msg::Pose& goal) {
    goal_pose_ = goal;
    return true;
}

bool RRTStar::set_start(const geometry_msgs::msg::Pose& start) {
    start_pose_ = start;
    // Clear previous tree and initialize with start node
    nodes_.clear();
    nodes_.push_back(std::make_shared<Node>(start_pose_));
    goal_reached_ = false;
    return true;
}

void RRTStar::set_step_size(double step) {
    step_size_ = step;
}

void RRTStar::set_max_iterations(int iter) {
    max_iterations_ = iter;
}

void RRTStar::set_search_radius(double radius) {
    search_radius_ = radius;
}

void RRTStar::set_area_bounds(double min_x, double max_x, 
                             double min_y, double max_y,
                             double min_z, double max_z) {
    min_x_ = min_x;
    max_x_ = max_x;
    min_y_ = min_y;
    max_y_ = max_y;
    min_z_ = min_z;
    max_z_ = max_z;
}

std::vector<geometry_msgs::msg::Pose> RRTStar::path_finding() {
    for (int i = 0; i < max_iterations_ && !goal_reached_; ++i) {
        add_one_more_Node();
    }
    
    return get_path();
}

void RRTStar::add_one_more_Node() {
    geometry_msgs::msg::Pose rand_point = get_random_point();
    std::shared_ptr<Node> nearest_node = find_nearest_node(rand_point);

    if (!nearest_node) return;

    geometry_msgs::msg::Pose new_pose = steer(nearest_node->pose, rand_point);

    if (is_collision_free(nearest_node->pose, new_pose)) {
        auto new_node = std::make_shared<Node>(new_pose);
        new_node->parent = nearest_node;
        new_node->cost_ = calculate_cost(new_node);

        auto near_nodes = find_near_nodes(new_node);

        for (const auto& near_node : near_nodes) {
            if (is_collision_free(near_node->pose, new_pose)) {
                double new_cost = near_node->cost_ + get_distance(near_node->pose, new_pose);
                if (new_cost < new_node->cost_) {
                    new_node->parent = near_node;
                    new_node->cost_ = new_cost;
                }
            }
        }

        nodes_.push_back(new_node);
        rewire(new_node, near_nodes);

        if (get_distance(new_pose, goal_pose_) <= step_size_) {
            goal_reached_ = true;
            auto goal_node = std::make_shared<Node>(goal_pose_);
            goal_node->parent = new_node;
            goal_node->cost_ = calculate_cost(goal_node);
            nodes_.push_back(goal_node);
        }

    } else {
        // If steer fails due to collision, mark nearest node as dead
        nearest_node->alive_ = false;
    }
}

std::vector<geometry_msgs::msg::Pose> RRTStar::get_path() {
    std::vector<geometry_msgs::msg::Pose> path;
    if (goal_reached_) {
        auto node = nodes_.back(); // Goal node
        while (node != nullptr) {
            path.push_back(node->pose);
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
    }
    
    return path;
}

geometry_msgs::msg::Pose RRTStar::get_random_point() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(min_x_, max_x_);
    std::uniform_real_distribution<> y_dist(min_y_, max_y_);
    std::uniform_real_distribution<> z_dist(min_z_, max_z_);
    
    geometry_msgs::msg::Pose point;
    point.position.x = x_dist(gen);
    point.position.y = y_dist(gen);
    point.position.z = z_dist(gen);
    return point;
}

std::shared_ptr<Node> RRTStar::find_nearest_node(const geometry_msgs::msg::Pose& point) {
    std::shared_ptr<Node> nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();

    for (const auto& node : nodes_) {
        if (!node->alive_) continue; // Skip dead nodes
        double dist = get_distance(node->pose, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node;
        }
    }

    return nearest;
}


geometry_msgs::msg::Pose RRTStar::steer(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to) {
    double dist = get_distance(from, to);
    
    geometry_msgs::msg::Pose new_pose;
    if (dist <= step_size_) {
        new_pose = to;
    } else {
        double ratio = step_size_ / dist;
        new_pose.position.x = from.position.x + (to.position.x - from.position.x) * ratio;
        new_pose.position.y = from.position.y + (to.position.y - from.position.y) * ratio;
        new_pose.position.z = from.position.z + (to.position.z - from.position.z) * ratio;
    }
    
    return new_pose;
}

bool RRTStar::is_collision_free(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to) {
    // Check if the points are within bounds
    if (to.position.x < min_x_ || to.position.x > max_x_ ||
        to.position.y < min_y_ || to.position.y > max_y_ ||
        to.position.z < min_z_ || to.position.z > max_z_) {
        return false;
    }
    
    // Check for collision with obstacles
    return !check_collision(from.position.x, from.position.y, from.position.z,
                           to.position.x, to.position.y, to.position.z);
}

std::vector<std::shared_ptr<Node>> RRTStar::find_near_nodes(const std::shared_ptr<Node>& new_node) {
    std::vector<std::shared_ptr<Node>> near_nodes;
    double radius = search_radius_ * std::pow(std::log(nodes_.size() + 1) / (nodes_.size() + 1), 1.0/3.0);
    
    for (const auto& node : nodes_) {
        if (get_distance(node->pose, new_node->pose) <= radius) {
            near_nodes.push_back(node);
        }
    }
    
    return near_nodes;
}

void RRTStar::rewire(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& near_nodes) {
    for (const auto& near_node : near_nodes) {
        if (near_node != new_node->parent && is_collision_free(new_node->pose, near_node->pose)) {
            double new_cost = new_node->cost_ + get_distance(new_node->pose, near_node->pose);
            if (new_cost < near_node->cost_) {
                near_node->parent = new_node;
                near_node->cost_ = new_cost;
            }
        }
    }
}

double RRTStar::calculate_cost(const std::shared_ptr<Node>& node) {
    if (node->parent == nullptr) {
        return 0.0;
    }
    return node->parent->cost_ + get_distance(node->parent->pose, node->pose);
}

double RRTStar::get_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
    double dx = a.position.x - b.position.x;
    double dy = a.position.y - b.position.y;
    double dz = a.position.z - b.position.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// void RRTStar::set_status(Node node, bool status) {
//     // node.status_ = status;
// }

void RRTStar::add_obstacle(const Obstacle& obstacle) {
    obstacles_.push_back(std::make_shared<Obstacle>(obstacle));
}

void RRTStar::clear_obstacles() {
    obstacles_.clear();
}

bool RRTStar::check_collision(double x1, double y1, double z1, 
                             double x2, double y2, double z2) const {
    // Check each obstacle for collision with the line segment
    for (const auto& obstacle : obstacles_) {
        // Check if either endpoint is inside the obstacle
        if (obstacle->collides(x1, y1, z1) || obstacle->collides(x2, y2, z2)) {
            return true;
        }
        
    }
    
    return false;
}

} // namespace rrt_star

