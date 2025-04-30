// ----------------------------------------------3d rviz
#include "path_planning/rrt_star.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <array>
#include <chrono>

class RRTStar3DVisualizer : public rclcpp::Node
{
public:
    RRTStar3DVisualizer() : Node("rrt_star_3d_visualizer")
    {
        // Create publishers for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_3d_markers", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_3d_path", 10);
        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_3d_obstacles", 10);
        tree_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_3d_tree", 10);

        // Create timers for visualization updates
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&RRTStar3DVisualizer::visualizeAll, this));

        // Run RRT* path planning
        runRRTStar3D();

        // Initialize 3D obstacles
        initialize3DObstacles();

    }

private:
    void initialize3DObstacles()
    {
        // Example 3D obstacles (x, y, z, length, width, height)
        obstacles_3d_ = {
            {0.0, 0.0, 0.0, 2.0, 2.0, 3.0},    // Central tall obstacle
            {3.0, 3.0, 2.0, 1.5, 1.5, 2.0},    // Near goal
            {-2.0, -2.0, -2.5, 2.0, 2.0, 1.5}  // Near start
        };
    }

    void runRRTStar3D()
    {
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

        // Configure planner for 3D
        rrtstar.set_goal(goal_pose);
        rrtstar.set_start(start_pose);
        rrtstar.set_step_size(0.5);
        rrtstar.set_max_iterations(10000);
        rrtstar.set_search_radius(2.0);
        rrtstar.set_area_bounds(-10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

        // Add 3D obstacles
        for (const auto& obs : obstacles_3d_) {
            rrtstar.add_obstacle(rrt_star::Obstacle(obs[0], obs[1], obs[2], obs[3], obs[4], obs[5]));
        }

        // Find path
        path_points_ = rrtstar.path_finding();
        RCLCPP_INFO(this->get_logger(), "Number of nodes in path: %zu", path_points_.size());

        if (!path_points_.empty()) {
            double total_distance = 0.0;
            for (size_t i = 0; i < path_points_.size() - 1; i++) {
                total_distance += rrtstar.get_distance(path_points_[i], path_points_[i+1]);
            }
            RCLCPP_INFO(this->get_logger(), "Total 3D path distance: %.2f meters", total_distance);
        }

        // Store start and goal points for visualization
        start_point_ = start_pose;
        goal_point_ = goal_pose;
    }

    void visualizeAll()
    {
        visualizeStartGoal();
        visualizePath();
        visualizeObstacles3D();
    }

    void visualizeStartGoal()
    {
        // Publish start point marker (green sphere)
        auto start_marker = createMarker(
            "start_point",
            visualization_msgs::msg::Marker::SPHERE,
            0,
            0.0, 1.0, 0.0, // Green
            start_point_.position.x,
            start_point_.position.y,
            start_point_.position.z,
            0.4);
        marker_pub_->publish(start_marker);

        // Publish goal point marker (red sphere)
        auto goal_marker = createMarker(
            "goal_point",
            visualization_msgs::msg::Marker::SPHERE,
            1,
            1.0, 0.0, 0.0, // Red
            goal_point_.position.x,
            goal_point_.position.y,
            goal_point_.position.z,
            0.4);
        marker_pub_->publish(goal_marker);
    }

    void visualizePath()
    {
        if (!path_points_.empty()) {
            // Create path marker (blue line strip)
            auto path_marker = createMarker(
                "path",
                visualization_msgs::msg::Marker::LINE_STRIP,
                2,
                0.0, 0.0, 1.0, // Blue
                0.0, 0.0, 0.0,
                0.1); // Line width

            path_marker.points.reserve(path_points_.size());
            for (const auto& pose : path_points_) {
                geometry_msgs::msg::Point p;
                p.x = pose.position.x;
                p.y = pose.position.y;
                p.z = pose.position.z;
                path_marker.points.push_back(p);
            }

            path_pub_->publish(path_marker);
        }
    }

    void visualizeObstacles3D()
    {
        int id = 10; // Starting ID for obstacles
        
        for (const auto& obs : obstacles_3d_) {
            auto obstacle_marker = createMarker(
                "obstacles",
                visualization_msgs::msg::Marker::CUBE,
                id++,
                1.0, 0.0, 1.0, // Magenta
                obs[0], obs[1], obs[2], // Position
                obs[3], obs[4], obs[5]  // Dimensions
            );
            obstacles_pub_->publish(obstacle_marker);
        }
    }

    visualization_msgs::msg::Marker createMarker(
        const std::string& ns,
        int32_t type,
        int32_t id,
        float r, float g, float b,
        float x, float y, float z,
        float scale_x, float scale_y = 0.0, float scale_z = 0.0)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.w = 1.0;

        // Handle uniform scaling if only one value provided
        if (scale_y == 0.0 && scale_z == 0.0) {
            marker.scale.x = scale_x;
            marker.scale.y = scale_x;
            marker.scale.z = scale_x;
        } else {
            marker.scale.x = scale_x;
            marker.scale.y = scale_y;
            marker.scale.z = scale_z;
        }

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 0.8; // Slightly transparent

        marker.lifetime = rclcpp::Duration::from_seconds(0.5); // Refresh every 0.5 seconds

        return marker;
    }

    // ROS 2 publishers and timers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Visualization data
    geometry_msgs::msg::Pose start_point_;
    geometry_msgs::msg::Pose goal_point_;
    std::vector<geometry_msgs::msg::Pose> path_points_;
    std::vector<std::array<double, 6>> obstacles_3d_; // x, y, z, length, width, height
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTStar3DVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}