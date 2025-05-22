/*#include <motion/Controller.hpp>
#include <chrono>
using namespace std::chrono_literals;

Controller::Controller():Node("Controller")
{
    using std::placeholders::_1;

    publisher_ = this-> create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel",10);
    sub_=this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Controller::execute_command,this,_1));
    count=0;
    
}

Controller::~Controller()
{
    printf("MATADO SIDO HE\n");
}


void Controller::execute_command(const turtlesim::msg::Pose msg)
{
    geometry_msgs::msg::Twist vel;
    
    //COSAS
    vel.linear.x = 2.0;
    vel.angular.z = 0.0;
    publisher_->publish(vel);

    rclcpp::sleep_for(std::chrono::seconds(1));

    vel.linear.x = 0.0;
    vel.angular.z = 0.0;

    publisher_->publish(vel);

    
}

int main (int argc, char * argv[])
{
    rclcpp::init (argc,argv);
    auto node=std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
    */

#include <motion/Controller.hpp>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/teleport_absolute.hpp"


using namespace std::chrono_literals;

void Controller::initialize()
{
    auto teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    while (!teleport_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio de teleport...");
    }

    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    teleport_request->x = 2.0;
    teleport_request->y = 7.0;
    teleport_request->theta = 0.0;

    auto teleport_future = teleport_client->async_send_request(teleport_request);
    rclcpp::spin_until_future_complete(shared_from_this(), teleport_future);

    auto pen_client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    while (!pen_client->wait_for_service(1s)) {
        RCLCPP_INFO(this->get_logger(), "Esperando servicio set_pen...");
    }

    auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
    pen_request->r = 255;
    pen_request->g = 0;
    pen_request->b = 0;
    pen_request->width = 5;
    pen_request->off = 1;

    auto pen_future = pen_client->async_send_request(pen_request);
    rclcpp::spin_until_future_complete(shared_from_this(), pen_future);
}



void set_pen_color(std::shared_ptr<rclcpp::Node> node, uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off)
{
    auto client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    // Espera que el servicio esté disponible
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(node->get_logger(), "Esperando al servicio /turtle1/set_pen...");
    }
    
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = off;

    auto result = client->async_send_request(request);
 
}

Controller::Controller() : Node("Controller"), count(0), has_moved(false)
{
    using std::placeholders::_1;
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&Controller::execute_command, this, _1));
}

Controller::~Controller()
{
    printf("MATADO SIDO HE\n");
}

void Controller::execute_command(const turtlesim::msg::Pose &msg)
{
    if (has_moved)
        return;

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

    set_pen_color(shared_from_this(), 255, 0, 0, 5, 0);  // Cambia a rojo
    rclcpp::sleep_for(100ms);        // Pequeña espera por si el servicio tarda

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

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
