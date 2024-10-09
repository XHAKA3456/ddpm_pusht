#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

bool planAndExecute(moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit::planning_interface::MoveGroupInterface::Plan& plan, rclcpp::Logger logger) {
    moveit::core::MoveItErrorCode success = move_group_interface.execute(plan);
    if (success == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(logger, "Execution successful!");
        return true;
    } else {
        RCLCPP_ERROR(logger, "Execution failed!");
        return false;
    }
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("moveit_trajectory_example");

    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "m0609_arm");

    auto const logger = rclcpp::get_logger("moveit_trajectory_example");

    // **1. 로봇을 "ready" 위치로 이동시킴**
    // move_group_interface.setNamedTarget("ready");
    // moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
    // if (!planAndExecute(move_group_interface, ready_plan, logger)) {
    //     RCLCPP_ERROR(logger, "Failed to move to 'ready' position.");
    //     rclcpp::shutdown();
    //     return 1;
    // }

    // **2. Cartesian 경로를 위한 웨이포인트 정의**
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Define the first waypoint
    geometry_msgs::msg::Pose waypoint1;
    waypoint1.position.x = 0.360;
    waypoint1.position.y = 0.079;
    waypoint1.position.z = 0.092;
    waypoint1.orientation.x = 1.0;
    waypoints.push_back(waypoint1); // Add waypoint1

    // Define the second waypoint
    geometry_msgs::msg::Pose waypoint2;
    waypoint2.position.x = 0.371;
    waypoint2.position.y = 0.087;
    waypoint2.position.z = 0.092;
    waypoint2.orientation.x = 1.0;
    waypoints.push_back(waypoint2); // Add waypoint2

    // Define the third waypoint
    geometry_msgs::msg::Pose waypoint3;
    waypoint3.position.x = 0.444;
    waypoint3.position.y = 0.067;
    waypoint3.position.z = 0.092;
    waypoint3.orientation.x = 1.0;
    waypoints.push_back(waypoint3); // Add waypoint3

    // geometry_msgs::msg::Pose waypoint4;
    // waypoint3.position.x = 0.445;
    // waypoint3.position.y = 0.065;
    // waypoint3.position.z = 0.095;
    // waypoint3.orientation.x = 0.0;
    // waypoint3.orientation.y = 1.0;
    // waypoint3.orientation.z = 0.0;
    // waypoint3.orientation.w = 0.0;
    // waypoints.push_back(waypoint4); // Add waypoint3    

    RCLCPP_INFO(logger, "Retrieved %zu waypoints:", waypoints.size());
    for (const auto& waypoint : waypoints) {
        RCLCPP_INFO(logger, "Waypoint - Position: [x: %.3f, y: %.3f, z: %.3f]",
                    waypoint.position.x,
                    waypoint.position.y,
                    waypoint.position.z );
    }

    // Cartesian 경로 계산 및 계획 생성
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;

    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 
        eef_step, 
        jump_threshold, 
        trajectory);

    if (fraction > 0.0) {
        RCLCPP_INFO(logger, "Cartesian path computed with fraction: %f", fraction);
        
        // Execute the computed Cartesian path
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory; // 계산된 경로를 계획에 설정
        planAndExecute(move_group_interface, cartesian_plan, logger);
    } else {
        RCLCPP_ERROR(logger, "Cartesian path computation failed!");
    }

    rclcpp::shutdown();
    return 0;
}