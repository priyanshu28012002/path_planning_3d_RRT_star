#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <algorithm>

namespace rrt_star {

    class Node {
        public:
            geometry_msgs::msg::Pose pose;
            std::shared_ptr<Node> parent;
            double cost_;
            bool alive_; // true = active, false = dead
        
            Node(const geometry_msgs::msg::Pose& pose_)
                : pose(pose_), parent(nullptr), cost_(0.0), alive_(true) {}
        };
class Obstacle {
public:
    Obstacle(double x, double y, double z, double length, double width, double height)
        : x_(x), y_(y), z_(z), length_(length), width_(width), height_(height) {}
    
    bool collides(double x, double y, double z) const {
        return (x > x_ - length_ / 2) && (x < x_ + length_ / 2) &&
               (y > y_ - width_ / 2) && (y < y_ + width_ / 2) &&
               (z > z_ - height_ / 2) && (z < z_ + height_ / 2);
    }

    // Getters
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }
    double length() const { return length_; }
    double width() const { return width_; }
    double height() const { return height_; }

private:
    double x_;      // Center x coordinate
    double y_;      // Center y coordinate
    double z_;      // Center z coordinate
    double length_; // Length of the obstacle (x-dimension)
    double width_;  // Width of the obstacle (y-dimension)
    double height_; // Height of the obstacle (z-dimension)
};

class RRTStar {
public:
    RRTStar();
    ~RRTStar();

    bool set_goal(const geometry_msgs::msg::Pose& goal);
    bool set_start(const geometry_msgs::msg::Pose& start);
    std::vector<geometry_msgs::msg::Pose> path_finding();
    void add_one_more_Node();
    std::vector<geometry_msgs::msg::Pose> get_path();

    // Configuration methods
    void set_step_size(double step);
    void set_max_iterations(int iter);
    void set_search_radius(double radius);
    void set_area_bounds(double min_x, double max_x, 
                        double min_y, double max_y,
                        double min_z, double max_z);
    double get_distance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b);
    void set_status(Node node, bool status);

    void add_obstacle(const Obstacle& obstacle);
    void clear_obstacles();

private:
    geometry_msgs::msg::Pose goal_pose_;
    geometry_msgs::msg::Pose start_pose_;
    std::vector<std::shared_ptr<Node>> nodes_;
    std::vector<std::shared_ptr<Obstacle>> obstacles_;
    bool check_collision(double x1, double y1, double z1, 
                        double x2, double y2, double z2) const;

    // Parameters
    double step_size_;
    int max_iterations_;
    double search_radius_;
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

    bool goal_reached_;

    // Helper methods
    geometry_msgs::msg::Pose get_random_point();
    std::shared_ptr<Node> find_nearest_node(const geometry_msgs::msg::Pose& point);
    geometry_msgs::msg::Pose steer(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to);
    bool is_collision_free(const geometry_msgs::msg::Pose& from, const geometry_msgs::msg::Pose& to);
    std::vector<std::shared_ptr<Node>> find_near_nodes(const std::shared_ptr<Node>& new_node);
    void rewire(std::shared_ptr<Node>& new_node, const std::vector<std::shared_ptr<Node>>& near_nodes);
    double calculate_cost(const std::shared_ptr<Node>& node);
};

} // namespace rrt_star

#endif // RRT_STAR_HPP