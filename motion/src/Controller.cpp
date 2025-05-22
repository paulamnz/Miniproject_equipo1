#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class Controller : public rclcpp::Node
{
public:
    Controller() : Node("Controller"), has_moved(false)
    {
        using namespace std::placeholders;

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&Controller::execute_command, this, _1));
    }

    void initialize()
    {
        // Teleport
        auto teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
        while (!teleport_client->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Esperando servicio de teleport...");
        }

        auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport_request->x = 5.0;
        teleport_request->y = 5.0;
        teleport_request->theta = 0.0;

        auto teleport_future = teleport_client->async_send_request(teleport_request);

        // Desactivar lápiz
        auto pen_client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
        while (!pen_client->wait_for_service(1s)) {
            RCLCPP_INFO(this->get_logger(), "Esperando servicio set_pen...");
        }

        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = 255;
        pen_request->g = 0;
        pen_request->b = 0;
        pen_request->width = 5;
        pen_request->off = 1;  // No pintar inicialmente

        auto pen_future = pen_client->async_send_request(pen_request);

        // Guardamos pen_client_ para activarlo luego
        pen_client_ = pen_client;
    }

private:
    void execute_command(const turtlesim::msg::Pose &msg)
    {
        if (has_moved) return;

        // Activar lápiz justo antes de moverse
        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = 255;
        pen_request->g = 0;
        pen_request->b = 0;
        pen_request->width = 5;
        pen_request->off = 0;  // ACTIVAR


        geometry_msgs::msg::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 1.6;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 2.0;
    vel.angular.z = 0.0;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 0.0;
    vel.angular.z = -1.6;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 2.0;
    vel.angular.z = -3.0;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 0.0;
    vel.angular.z = 2.5;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 1.0;
    vel.angular.z = 0.0;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    publisher_->publish(vel);
    rclcpp::sleep_for(1s);

        has_moved = true;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    bool has_moved;
};

// main incluido en el mismo archivo
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
