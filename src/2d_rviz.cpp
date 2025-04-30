#include "path_planning/rrt_star.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RRTStarVisualizer : public rclcpp::Node
{
public:
    RRTStarVisualizer() : Node("rrt_star_visualizer")
    {
        // Create a publisher for visualization markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_markers", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_path", 10);
        obstacles_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rrt_star_obstacle", 10);
        // Create a timer to periodically publish markers
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RRTStarVisualizer::visualizeAll, this));

        timer_point_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RRTStarVisualizer::timer_point_callback_, this));

        timer_path_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RRTStarVisualizer::visualizeObstacles, this));

        // Run RRT* path planning
        runRRTStar();

        // Initialize obstacles (you can modify these or add a way to set them dynamically)
        initializeObstacles();
    }

private:
    void initializeObstacles()
    {
        // Example obstacles (position x, y, size x, y)
        obstacles_ = {
            {0.0, 0.0, 1.0, 1.0},  // Center obstacle
            {1.5, 1.5, 0.5, 0.5},  // Near goal
            {-1.0, -1.0, 0.8, 0.8} // Near start
        };
    }

    void runRRTStar()
    {
        rrt_star::RRTStar rrtstar;
        geometry_msgs::msg::Pose golepose_;
        golepose_.position.x = 4;
        golepose_.position.y = 4;
        geometry_msgs::msg::Pose startpose_;
        startpose_.position.x = -4;
        startpose_.position.y = -4;

        // Set obstacles in RRT* (assuming your RRTStar class has this capability)
        // for (const auto& obs : obstacles_) {
        //     rrtstar.add_obstacle(obs[0], obs[1], obs[2], obs[3]);
        // }

        rrtstar.set_goal(golepose_);
        rrtstar.set_start(startpose_);
        rrtstar.set_max_iterations(10000);

        std::vector<geometry_msgs::msg::Pose> paths = rrtstar.path_finding();
        RCLCPP_INFO(this->get_logger(), "Number of nodes in path: %zu", paths.size());

        if (!paths.empty())
        {
            double total_distance_ = 0.0;
            for (size_t i = 0; i < paths.size() - 1; i++)
            {
                total_distance_ += rrtstar.get_distance(paths[i], paths[i + 1]);
            }
            RCLCPP_INFO(this->get_logger(), "Total path distance: %.2f", total_distance_);
        }

        // Store the points for visualization
        start_point_ = startpose_;
        goal_point_ = golepose_;
        path_points_ = paths;
    }

    void visualizeAll()
    {
        visualizePoints();
    }

    void timer_point_callback_()
    {
        visGole();
        visStart();
    }

    void visStart()
    {
        auto start_marker = createMarker(
            "start_point",
            visualization_msgs::msg::Marker::SPHERE,
            0,             // ID
            0.0, 1.0, 0.0, // Green color
            start_point_.position.x,
            start_point_.position.y,
            0.0,
            0.3); // Scale
        marker_pub_->publish(start_marker);
    }
    void visGole()
    {
        // Publish goal point marker
        auto goal_marker = createMarker(
            "goal_point",
            visualization_msgs::msg::Marker::SPHERE,
            1,             // ID
            1.0, 0.0, 0.0, // Red color
            goal_point_.position.x,
            goal_point_.position.y,
            0.0,
            0.3); // Scale

        std::this_thread::sleep_for(std::chrono::microseconds(10));
        marker_pub_->publish(goal_marker);
    }
    void visualizePoints()
    {
        // If we have a path, publish it
        if (!path_points_.empty())
        {
            auto path_marker = createMarker(
                "rrt_star_path",
                visualization_msgs::msg::Marker::LINE_STRIP,
                2,             // ID
                0.0, 0.0, 1.0, // Blue color
                0.0, 0.0, 0.0,
                0.05); // Line width

            path_marker.points.reserve(path_points_.size());
            for (const auto &pose : path_points_)
            {
                geometry_msgs::msg::Point p;
                p.x = pose.position.x;
                p.y = pose.position.y;
                p.z = 0.0;
                path_marker.points.push_back(p);
            }

            path_pub_->publish(path_marker);
        }
    }

    void visualizeObstacles()
    {
        int obstacle_id = 99; // Start ID for obstacles (to avoid conflict with other markers)

        for (const auto &obs : obstacles_)
        {
            auto obstacle_marker = createMarker(
                "obstacles",
                visualization_msgs::msg::Marker::CUBE,
                obstacle_id++,
                1.0, 0.0, 1.0,       // Magenta color
                obs[0], obs[1], 0.0, // Position
                obs[2], obs[3], 0.1  // Scale (x, y, z)
            );

            obstacles_pub_->publish(obstacle_marker);
        }
    }

    visualization_msgs::msg::Marker createMarker(
        const std::string &ns,
        int32_t type,
        int32_t id,
        float r, float g, float b,
        float x, float y, float z,
        float scale_x, float scale_y, float scale_z)
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

        marker.scale.x = scale_x;
        marker.scale.y = scale_y;
        marker.scale.z = scale_z;

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        marker.lifetime = rclcpp::Duration::from_seconds(1.1);

        return marker;
    }

    // Overloaded version for uniform scaling
    visualization_msgs::msg::Marker createMarker(
        const std::string &ns,
        int32_t type,
        int32_t id,
        float r, float g, float b,
        float x, float y, float z,
        float scale)
    {
        return createMarker(ns, type, id, r, g, b, x, y, z, scale, scale, scale);
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacles_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_point_;
    rclcpp::TimerBase::SharedPtr timer_path_;

    geometry_msgs::msg::Pose start_point_;
    geometry_msgs::msg::Pose goal_point_;
    std::vector<geometry_msgs::msg::Pose> path_points_;
    std::vector<std::array<double, 4>> obstacles_; // x, y, size_x, size_y
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RRTStarVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

