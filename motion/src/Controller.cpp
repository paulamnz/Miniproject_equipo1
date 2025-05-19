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
using namespace std::chrono_literals;

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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
