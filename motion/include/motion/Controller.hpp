#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"  // Añadido para usar el servicio SetPen

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

    // Clientes para servicios
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    std::shared_ptr<turtlesim::srv::SetPen::Request> pen_request_;
};

#endif  // MOTION_CONTROLLER_HPP_
