// include/motion/Controller.hpp
#ifndef MOTION_CONTROLLER_HPP_
#define MOTION_CONTROLLER_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/set_pen.hpp"

/// Control básico de movimiento + dibujo para turtlesim
typedef std::shared_ptr<rclcpp::Node> NodePtr;
class Controller
{
public:
  explicit Controller(NodePtr node);

  // primitivas
  void pen(uint8_t r, uint8_t g, uint8_t b, uint8_t width = 2, bool off = false);
  void forward(double distance, double speed = 1.0);
  void rotate(double radians, double angular_speed = 1.0);
  void stop();

  // figuras
  void drawHexagon(double side);
  void drawTriangle(double side, bool left);
  void drawRectangle(double base, double height);

  // escena completa
  void drawScene();

private:
  NodePtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

#endif  // MOTION_CONTROLLER_HPP_