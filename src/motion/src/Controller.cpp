// src/Controller.cpp
#include "motion/Controller.hpp"
#include <cmath>      // M_PI
#include <chrono>     // duraciones

using namespace std::chrono_literals;

Controller::Controller(NodePtr node)
: node_(node)
{
  // Creamos el publicador de velocidad para la tortuga (/turtle1/cmd_vel)
  pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
    "/turtle1/cmd_vel", 10);
}

void Controller::pen(uint8_t r, uint8_t g, uint8_t b,
                     uint8_t width, bool off)
{
  // Cliente al servicio /turtle1/set_pen para cambiar color/grosor
  auto client = node_->create_client<turtlesim::srv::SetPen>(
    "/turtle1/set_pen");
  // Espera hasta que el servicio esté disponible
  while (!client->wait_for_service(1s) && rclcpp::ok()) {
    RCLCPP_INFO(node_->get_logger(),
                "Esperando /turtle1/set_pen...");
  }
  // Preparamos la petición
  auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
  req->r = r;      // componente rojo
  req->g = g;      // componente verde
  req->b = b;      // componente azul
  req->width = width;  // grosor
  req->off = off;      // si true, levanta el lápiz

  // Enviamos la petición y esperamos la respuesta
  auto fut = client->async_send_request(req);
  rclcpp::spin_until_future_complete(node_, fut);
}

void Controller::forward(double distance, double speed)
{
  // Creamos el mensaje de velocidad lineal
  geometry_msgs::msg::Twist cmd;
  // Asignar sentido: positivo o negativo según distance
  cmd.linear.x = (distance >= 0 ? speed : -speed);
  // Tiempo total = distancia / velocidad
  double duration = std::abs(distance / speed);

  auto start = node_->now();
  rclcpp::Rate rate(50);  // bucle a 50 Hz
  // Publicar cmd hasta completar el tiempo deseado
  while ((node_->now() - start).seconds() < duration) {
    pub_->publish(cmd);
    rate.sleep();
  }
  stop();               // asegurar parada
  rclcpp::sleep_for(50ms);  // breve pausa
}

void Controller::rotate(double radians, double angular_speed)
{
  // Mensaje de velocidad angular
  geometry_msgs::msg::Twist cmd;
  cmd.angular.z = (radians >= 0 ? angular_speed : -angular_speed);
  double duration = std::abs(radians / angular_speed);

  auto start = node_->now();
  rclcpp::Rate rate(50);
  // Similar a forward(), pero con giro
  while ((node_->now() - start).seconds() < duration) {
    pub_->publish(cmd);
    rate.sleep();
  }
  stop();
  rclcpp::sleep_for(50ms);
}

void Controller::stop()
{
  // Publicar mensaje con valores cero para detener
  pub_->publish(geometry_msgs::msg::Twist{});
}

void Controller::drawHexagon(double side)
{
  // Seis lados y seis giros de 60° (π/3 rad)
  for (int i = 0; i < 6; ++i) {
    forward(side);
    rotate(M_PI / 3.0);
  }
}

void Controller::drawTriangle(double side, bool left)
{
  // Giros de 120°: signo depende de 'left' para orientar vértice
  double turn = (left ? 2 * M_PI / 3.0 : -2 * M_PI / 3.0);
  for (int i = 0; i < 3; ++i) {
    forward(side);
    rotate(turn);
  }
}

void Controller::drawRectangle(double base, double height)
{
  // Dos veces: base - altura - base - altura
  for (int i = 0; i < 2; ++i) {
    forward(base);
    rotate(M_PI / 2);      // 90° a la derecha
    forward(height);
    rotate(M_PI / 2);
  }
}

void Controller::drawScene()
{
  // 1) Movernos 2.5 m hacia la izquierda (miramos al oeste)
  rotate(M_PI);        // orientación oeste
  forward(2.5);
  rotate(M_PI);        // vuelta a este para dibujar

  // 2) Configurar lápiz y dibujar primer triángulo
  pen(0, 0, 255, 3, false);            // lápiz azul grueso
  drawTriangle(1.5, true);            // triángulo equilátero
  rclcpp::sleep_for(1s);               // pausa de 1 segundo

  // 3) Avanzar 1.5 m en X y dibujar hexágono
  forward(1.5);
  drawHexagon(2.0);                    // lado 2 m
  rclcpp::sleep_for(1s);

  // 4) Avanzar 2 m en X
  forward(2.0);

  // 5) Dibujar segundo triángulo a la derecha
  drawTriangle(1.5, true);

  RCLCPP_INFO(node_->get_logger(),
              "Dibujo secuencial completado.");
}

int main(int argc, char **argv)
{
  // Inicializar ROS2 y crear nodo
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("shape_drawer");
  Controller ctrl(node);

  // Ejecutar la rutina de dibujo
  ctrl.drawScene();

  // Finalizar ROS2
  rclcpp::shutdown();
  return 0;
}
