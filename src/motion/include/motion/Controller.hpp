// include/motion/Controller.hpp
#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/msg/pose.hpp"

/// Control bÃ¡sico de movimiento + dibujo para turtlesim
typedef std::shared_ptr<rclcpp::Node> NodePtr;
class Controller
{
public:
  explicit Controller(NodePtr node);

  void execute_command(const turtlesim::msg::Pose &msg);

  // primitivas
  void pen(uint8_t r, uint8_t g, uint8_t b, uint8_t width = 2, bool off = false);
  void forward(double distance, double speed = 2.0);
  void rotate(double radians, double angular_speed = 2.0);
  void stop();

  // figuras
  void drawHexagonFlat(double side);
  void drawTriangleUP(double side, bool left);
  void drawRectangleAboveHexagon(double base, double height);
  void drawCurves(double radius,
                  double radians,
                  std::string direction);

  // escena completa
  void drawScene();

private:

  double linear_speed_ = 2.0;
  NodePtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
};

#endif // MOTION_CONTROLLER_HPP_