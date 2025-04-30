// ------------------------------------------2d  
#include "path_planning/rrt_star.hpp"
#include <iostream>

int main(){
    rrt_star::RRTStar rrtstar;
    geometry_msgs::msg::Pose golepose_;
    golepose_.position.x = 2;
    golepose_.position.y = 2;
    geometry_msgs::msg::Pose startpose_;
    startpose_.position.x = -2;
    startpose_.position.y = -2;

    rrtstar.set_goal(golepose_);
    rrtstar.set_start(startpose_);
    rrtstar.set_max_iterations(10000);
    

    rrtstar.add_obstacle(rrt_star::Obstacle(0.0, 0.0, 0.0, 2.0, 4.0, 0.0));  // Central obstacle
    rrtstar.add_obstacle(rrt_star::Obstacle(3.0, 3.0, 0.0, 1.5, 1.5, 0.0));  // Near goal
    rrtstar.add_obstacle(rrt_star::Obstacle(-2.0, -2.0, 0.0, 3.0, 3.0, 0.0)); // Near start

    std::vector<geometry_msgs::msg::Pose> paths = rrtstar.path_finding();
    std::cout<<"No of node is "<<paths.size()<<std::endl;

    if(paths.size()!=0){
        double total_distance_ = 0.0;
        for(int i=0; i<paths.size()-1;i++){
            total_distance_ += rrtstar.get_distance(paths[i],paths[i+1] );
        }
        std::cout<<"total_distance_ "<<total_distance_<<std::endl;
    }

    return 0;
}
