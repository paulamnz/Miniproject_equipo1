/*#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class Controller : public rclcpp::Node
{
    public:
        Controller();
        ~Controller();
        void execute_command(const turtlesim::msg::Pose msg);

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
        size_t count;
        turtlesim::msg::Pose pose;

};*/

#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

class Controller : public rclcpp::Node
{
public:
    Controller();
    ~Controller();

    void execute_command(const turtlesim::msg::Pose &msg);

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    size_t count;
    turtlesim::msg::Pose pose;
    bool has_moved;
};

#endif  // MOTION_CONTROLLER_HPP_
