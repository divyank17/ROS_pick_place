#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class PickAndPlace : public rclcpp::Node
{
public:
    PickAndPlace() : Node("pick_and_place_node")
    {
        try
        {
            // Initialize the MoveGroupInterface for the manipulator group
            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator");
            RCLCPP_INFO(this->get_logger(), "MoveGroupInterface initialized successfully");

            // Set the maximum velocity and acceleration scaling factors
            move_group_->setMaxVelocityScalingFactor(0.1);
            move_group_->setMaxAccelerationScalingFactor(0.1);

            // Perform pick and place
            performPickAndPlace();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

private:
    void performPickAndPlace()
    {
        // Define the pick pose
        geometry_msgs::msg::Pose pick_pose;
        pick_pose.position.x = 0.4;
        pick_pose.position.y = 0.0;
        pick_pose.position.z = 0.4;
        pick_pose.orientation.w = 1.0;

        // Move to the pick pose
        move_group_->setPoseTarget(pick_pose);
        auto result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Pick operation successful");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Pick operation failed");
        }

        // Define the place pose
        geometry_msgs::msg::Pose place_pose;
        place_pose.position.x = 0.6;
        place_pose.position.y = 0.0;
        place_pose.position.z = 0.4;
        place_pose.orientation.w = 1.0;

        // Move to the place pose
        move_group_->setPoseTarget(place_pose);
        result = move_group_->move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Place operation successful");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Place operation failed");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<PickAndPlace>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
