#include "path_planning/rrt_star.hpp"
#include <iostream>
#include <memory>

int main() {
    // Create RRT* planner instance
    rrt_star::RRTStar rrtstar;

    // Set 3D goal position
    geometry_msgs::msg::Pose goal_pose;
    goal_pose.position.x = 5.0;
    goal_pose.position.y = 5.0;
    goal_pose.position.z = 5.0;

    // Set 3D start position
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = -5.0;
    start_pose.position.y = -5.0;
    start_pose.position.z = -5.0;

    // Configure planner
    rrtstar.set_goal(goal_pose);
    rrtstar.set_start(start_pose);
    rrtstar.set_step_size(0.5);
    rrtstar.set_max_iterations(5000);
    rrtstar.set_search_radius(2.0);
    rrtstar.set_area_bounds(-10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

    // Add 3D cuboid obstacles (x, y, z, length, width, height)
    rrtstar.add_obstacle(rrt_star::Obstacle(0.0, 0.0, 0.0, 2.0, 4.0, 3.0));  // Central obstacle
    rrtstar.add_obstacle(rrt_star::Obstacle(3.0, 3.0, 3.0, 1.5, 1.5, 4.0));  // Near goal
    rrtstar.add_obstacle(rrt_star::Obstacle(-2.0, -2.0, -2.0, 3.0, 3.0, 2.0)); // Near start

    // Find path
    std::vector<geometry_msgs::msg::Pose> path = rrtstar.path_finding();

    // Output results
    std::cout << "Path planning results:" << std::endl;
    std::cout << "Number of nodes in path: " << path.size() << std::endl;

    if (!path.empty()) {
        std::cout << "Path coordinates:" << std::endl;
        double total_distance = 0.0;
        
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << "Node " << i << ": (" 
                      << path[i].position.x << ", " 
                      << path[i].position.y << ", "
                      << path[i].position.z << ")" << std::endl;
            
            if (i > 0) {
                total_distance += rrtstar.get_distance(path[i-1], path[i]);
            }
        }
        
        std::cout << "Total path distance: " << total_distance << std::endl;
    } else {
        std::cout << "No path found to the goal!" << std::endl;
    }

    return 0;
}