#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <iomanip>

float zero_x = 0.334;
float zero_y = -0.113;
float zero_z = 0.092;

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

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber()
        : Node("pose_subscriber_node"), message_received_(false)
    {
        // Create a subscription to the "action_list" topic
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "action_list", 10, [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
                this->topic_callback(msg);
            });
    }

    bool has_message_received() const
    {
        return message_received_;
    }

    const std::vector<geometry_msgs::msg::Pose>& get_poses() const
    {
        return poses_;
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());

        poses_.clear();

        for (const auto& pose : msg->poses)
        {
            geometry_msgs::msg::Pose processed_pose;
            processed_pose.position.x = zero_x + pose.position.x;
            processed_pose.position.y = zero_y + pose.position.y;
            processed_pose.position.z = zero_z;

            processed_pose.orientation.w = 0.0;
            processed_pose.orientation.x = 1.0;
            processed_pose.orientation.y = 0.0;
            processed_pose.orientation.z = 0.0;


            poses_.push_back(processed_pose);

            RCLCPP_INFO(this->get_logger(), 
                "Pose - Position: [x: %.3f, y: %.3f, z: %.3f]", 
                processed_pose.position.x, 
                processed_pose.position.y, 
                processed_pose.position.z);
        }

        message_received_ = true;
    }

    bool message_received_;
    std::vector<geometry_msgs::msg::Pose> poses_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    using moveit::planning_interface::MoveGroupInterface;
    auto node = std::make_shared<rclcpp::Node>("moveit_ready_node");
    auto move_group_interface = MoveGroupInterface(node, "m0609_arm");
    auto const logger = rclcpp::get_logger("hello_moveit");

    if (!move_group_interface.startStateMonitor()) {
        RCLCPP_ERROR(logger, "Failed to start state monitor.");
        rclcpp::shutdown();
        return 1;
    }

    move_group_interface.setNamedTarget("ready");

    moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
    bool ready_success = static_cast<bool>(move_group_interface.plan(ready_plan));
    if (ready_success) {
        RCLCPP_INFO(logger, "Plan to 'ready' state was successful.");
        planAndExecute(move_group_interface, ready_plan, logger);
    } else {
        RCLCPP_ERROR(logger, "Planning to 'ready' state failed!");
        rclcpp::shutdown();
        return 1;
    }

    auto pose_subscriber_node = std::make_shared<PoseSubscriber>();

    auto future = std::async(std::launch::async, [&pose_subscriber_node]() {
        while (!pose_subscriber_node->has_message_received()) {
            rclcpp::spin_some(pose_subscriber_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    future.wait();

    const auto& waypoints = pose_subscriber_node->get_poses();

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

    move_group_interface.setNamedTarget("home");

    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    bool home_success = static_cast<bool>(move_group_interface.plan(home_plan));
    if (home_success) {
        RCLCPP_INFO(logger, "Plan to 'home' state was successful.");
        planAndExecute(move_group_interface, home_plan, logger);
    } else {
        RCLCPP_ERROR(logger, "Planning to 'home' state failed!");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
