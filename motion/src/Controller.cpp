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

/*void Controller::initialize()
{
    auto teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    teleport_request->x = 2.0;
    teleport_request->y = 7.0;
    teleport_request->theta = 0.0;
    set_pen_color(shared_from_this(), 255, 0, 0, 5, 1);
    auto teleport_future = teleport_client->async_send_request(teleport_request);
    set_pen_color(shared_from_this(), 255, 0, 0, 5, 0);
}



void set_pen_color(std::shared_ptr<rclcpp::Node> node, uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off)
{
    auto client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = off;

    auto result = client->async_send_request(request);
 
}*/

Controller::Controller() : Node("Controller"), count(0), has_moved(false) // se ejecuta al hacer launch por primera vez 
{
    using std::placeholders::_1;
    
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    sub_ = this->create_subscription<turtlesim::msg::Pose>(
        "/turtle1/pose", 10, std::bind(&Controller::execute_command, this, _1));

    //setpen
    auto client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 255;
    request->g = 0;
    request->b = 0;
    request->width = 5;
    request->off = 1;

    auto result = client->async_send_request(request);

    //teletransporte
    auto teleport_client = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");
    while (!teleport_client->wait_for_service(1s)) {
    
    if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrumpido mientras esperaba el servicio de teleport.");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Esperando al servicio de teleport...");
}

    auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
    teleport_request->x = 5.0;
    teleport_request->y = 5.0;
    teleport_request->theta = 0.0;
    result = client->async_send_request(request);
    auto teleport_future = teleport_client->async_send_request(teleport_request); //result del teleport
    request->off = 0;
    result = client->async_send_request(request);
}

Controller::~Controller()
{
    printf("MATADO SIDO HE\n");
}

void Controller::execute_command(const turtlesim::msg::Pose &msg)
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = 255;
    request->g = 0;
    request->b = 0;
    request->width = 5;
    request->off = 0;
    auto client = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
    auto result = client->async_send_request(request);
    if (has_moved)
        return;

    geometry_msgs::msg::Twist vel;



            // === R ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = 1.5; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = -1.2; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = -1.5; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(700s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === O ===
        vel.linear.x = 1.5; vel.angular.z = 2.2; publisher_->publish(vel); rclcpp::sleep_for(3s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === B ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1200ms);
        vel.linear.x = 0.0; vel.angular.z = -3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 2.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = 3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 2.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === O ===
        vel.linear.x = 1.5; vel.angular.z = 2.2; publisher_->publish(vel); rclcpp::sleep_for(3s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === R ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = 1.5; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = -1.2; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = -1.5; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(700ms);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === E ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = -1.57; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = 3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = -1.57; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = 3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);

        // === S ===
        vel.linear.x = 1.0; vel.angular.z = 2.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = -2.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === C ===
        vel.linear.x = 1.0; vel.angular.z = 2.0; publisher_->publish(vel); rclcpp::sleep_for(1500ms);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === U ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = -2.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);

        // === E (final) ===
        vel.linear.x = 2.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 0.0; vel.angular.z = -1.57; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = 3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = -1.57; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500ms);
        vel.linear.x = 0.0; vel.angular.z = 3.14; publisher_->publish(vel); rclcpp::sleep_for(1s);
        vel.linear.x = 1.0; vel.angular.z = 0.0; publisher_->publish(vel); rclcpp::sleep_for(500s);

        // Parar
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        publisher_->publish(vel);

    has_moved = true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
