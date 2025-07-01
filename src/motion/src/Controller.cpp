// src/Controller.cpp
#include "motion/Controller.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>  // M_PI
#include <chrono> // duraciones

using namespace std::chrono_literals;

// Prototipo Funciones auxiliares
double euclidean_distance(double x1, double y1, double x2, double y2);
double normalize_angle(double angle);
void rotate_to(double target_angle, rclcpp::Node::SharedPtr node,
			   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);
void move_to(double x_goal, double y_goal,
			 rclcpp::Node::SharedPtr node,
			 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub);

turtlesim::msg::Pose pose; // Variable global para guardar la pose actual

Controller::Controller(NodePtr node)
	: node_(node)
{
	using std::placeholders::_1;

	// Creamos el publicador de velocidad para la tortuga (/turtle1/cmd_vel)
	pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

	// Creamos el subscriptor de pose para la tortuga (/turtle1/pose)
	sub_ = node_->create_subscription<turtlesim::msg::Pose>(
		"/turtle1/pose", 10, std::bind(&Controller::execute_command, this, _1));
}

void Controller::execute_command(const turtlesim::msg::Pose &msg)
{
	pose = msg; // Actualizamos la pose global
}

void Controller::pen(uint8_t r, uint8_t g, uint8_t b, uint8_t width, bool off)
{
	// Cliente al servicio /turtle1/set_pen para cambiar color/grosor
	auto client = node_->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

	while (!client->wait_for_service(1s) && rclcpp::ok())
	{
		RCLCPP_INFO(node_->get_logger(), "Esperando /turtle1/set_pen...");
	}

	auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
	req->r = r;
	req->g = g;
	req->b = b;
	req->width = width;
	req->off = off;

	auto fut = client->async_send_request(req);
	rclcpp::spin_until_future_complete(node_, fut);
}

void Controller::forward(double distance, double speed)
{
	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = (distance >= 0 ? speed : -speed);

	double start_x = pose.x;
	double start_y = pose.y;

	rclcpp::Rate rate(50);
	while (rclcpp::ok() && euclidean_distance(start_x, start_y, pose.x, pose.y) < std::abs(distance))
	{
		pub_->publish(cmd);
		rclcpp::spin_some(node_);
		rate.sleep();
	}
	stop();
	rclcpp::sleep_for(50ms);
}

void Controller::rotate(double radians, double angular_speed)
{
	geometry_msgs::msg::Twist cmd;
	rclcpp::Rate rate(50);

	double start_angle = pose.theta;
	double target_angle = normalize_angle(start_angle + radians);

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node_);
		double error = normalize_angle(target_angle - pose.theta);

		if (std::abs(error) < 0.01)
			break;

		cmd.angular.z = (error > 0 ? 1.0 : -1.0) * angular_speed;
		pub_->publish(cmd);
		rate.sleep();
	}

	stop();
	rclcpp::sleep_for(50ms);
}

void Controller::stop()
{
	pub_->publish(geometry_msgs::msg::Twist{});
}

// ================= FUNCIONES DE DIBUJO =====================

void Controller::drawHexagonFlat(double side)
{
	// Guardar el punto de partida
	double x0 = pose.x;
	double y0 = pose.y;

	// Asegurarnos de empezar mirando al este
	rotate_to(0, node_, pub_);

	// Bajar lápiz para dibujar
	pen(0, 255, 0, 2, false);

	// Recorremos las 6 aristas del hexágono
	for (int i = 0; i < 6; ++i)
	{
		// Ángulo absoluto de esta arista (hexágono plano)
		double angle = normalize_angle(i * M_PI / 3.0);

		// Orientarnos hacia ese ángulo
		rotate_to(angle, node_, pub_);

		// Calcular el siguiente vértice
		double xt = x0 + side * std::cos(angle);
		double yt = y0 + side * std::sin(angle);

		// Movernos exactamente a ese vértice
		move_to(xt, yt, node_, pub_);

		// Actualizar punto de partida
		x0 = xt;
		y0 = yt;
	}

	// Levantar lápiz al terminar
	pen(255, 0, 0, 2, true);

	// Opcional: volver a orientar al este
	rotate_to(0, node_, pub_);
}

void Controller::drawTriangleUP(double side, bool left)
{
	rotate_to(0, node_, pub_);
	if (left)
	{
		rotate_to(2 * M_PI / 3, node_, pub_);
		forward(side);
		rotate_to(4 * M_PI / 3, node_, pub_);
		forward(side);
		rotate_to(0, node_, pub_);
		forward(side);
	}
	else
	{
		forward(side);
		rotate_to(2 * M_PI / 3, node_, pub_);
		forward(side);
		rotate_to(4 * M_PI / 3, node_, pub_);
		forward(side);
	}
	stop();
}

void Controller::drawRectangleAboveHexagon(double base, double height)
{
	// 0. Asegurarnos de llevar el lápiz levantado mientras nos movemos
	pen(255, 0, 0, 2, true);

	// 1. Calcular la altura hasta el vértice superior del hexágono (apotema * 2)
	//    apotema = (√3/2) * base
	double hex_apex = std::sqrt(3.0) * base;
	double climb = hex_apex;

	// 2. Subir sin dibujar hasta justo por encima del vértice del hexágono
	rotate_to(M_PI / 2, node_, pub_); // mirar norte
	forward(climb);

	// 3. Encender lápiz para empezar el rectángulo
	pen(0, 0, 255, 2, false);

	// 4. Lado superior del rectángulo: subir (norte)
	rotate_to(M_PI / 2, node_, pub_);
	forward(height);

	// 5. Lado izquierdo del rectángulo: girar a la izquierda (oeste)
	rotate_to(M_PI, node_, pub_);
	forward(base);

	// 6. Lado inferior del rectángulo: bajar (sur)
	rotate_to(-M_PI / 2, node_, pub_);
	forward(height);

	// 7. Lado derecho del rectángulo: girar a la derecha (este)
	rotate_to(0, node_, pub_);
	forward(base);

	// 8. Levantar lápiz al terminar
	pen(255, 0, 0, 2, true);
}

void Controller::drawCurves(double radius, double radians, std::string direction)
{
	double target_angle = radians;
	double linear = 1.0;
	double angular;

	if (direction == "up")
		angular = linear / radius;
	else if (direction == "down")
		angular = -linear / radius;
	else
		return;

	geometry_msgs::msg::Twist cmd;
	cmd.linear.x = linear;
	cmd.angular.z = angular;

	rclcpp::Rate rate(50);
	double last_theta = pose.theta;
	double accumulated_angle = 0;

	while (rclcpp::ok() && accumulated_angle < target_angle)
	{
		rclcpp::spin_some(node_);
		pub_->publish(cmd);

		double delta_theta = normalize_angle(pose.theta - last_theta);
		accumulated_angle += std::abs(delta_theta);
		last_theta = pose.theta;

		rate.sleep();
	}

	stop();
	rclcpp::sleep_for(50ms);
}

// ================= FUNCIONES DE LETRAS =====================

void Controller::drawLetterR(double x)
{
	move_to(x, 8.5, node_, pub_);
	rotate_to(M_PI / 2, node_, pub_);
	pen(0, 255, 0, 2, false);
	forward(2, 1);
	rotate_to(M_PI / 16, node_, pub_);
	drawCurves(0.5, M_PI, "down");
	rotate_to(-M_PI * 3 / 8, node_, pub_);
	forward(1.1, 1);
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterO(double x)
{
	move_to(x, 8.5, node_, pub_);
	rotate_to(0, node_, pub_);
	pen(0, 255, 0, 2, false);
	drawCurves(0.5, 2 * M_PI, "up");
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterB(double x)
{
	move_to(x, 8.5, node_, pub_);
	rotate_to(M_PI / 2, node_, pub_);
	pen(0, 255, 0, 2, false);
	forward(2, 1);
	rotate_to(-M_PI / 2, node_, pub_);
	forward(1, 1);
	rotate_to(M_PI / 16, node_, pub_);
	drawCurves(0.5, M_PI, "down");
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterE(double x)
{
	move_to(x, 9, node_, pub_);
	rotate_to(0, node_, pub_);
	pen(0, 255, 0, 2, false);
	forward(0.85, 1);
	rotate_to(M_PI / 2, node_, pub_);
	drawCurves(0.5, M_PI * 13 / 8, "up");
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterS(double x)
{
	move_to(x, 8.55, node_, pub_);
	rotate_to(-M_PI / 4, node_, pub_);
	pen(0, 255, 0, 2, false);
	drawCurves(0.25, M_PI, "up");
	drawCurves(0.25, M_PI * 9 / 8, "down");
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterC(double x)
{
	move_to(x, 9, node_, pub_);
	rotate_to(M_PI / 2, node_, pub_);
	pen(0, 255, 0, 2, false);
	drawCurves(0.5, M_PI / 5, "up");
	drawCurves(0.5, M_PI * 11 / 8, "up");
	pen(255, 0, 0, 2, true);
}

void Controller::drawLetterU(double x)
{
	move_to(x, 9.5, node_, pub_);
	rotate_to(-M_PI / 2, node_, pub_);
	pen(0, 255, 0, 2, false);
	forward(0.5, 1);
	drawCurves(0.4, M_PI * 7 / 8, "up");
	rotate_to(M_PI / 2, node_, pub_);
	forward(0.45, 1);
	pen(255, 0, 0, 2, true);
}

// ================= FUNCIONES DE ESCENA COMPLETA =====================

void Controller::drawScene()
{
	double side = 1.5;
	double x = 0.7;
	rclcpp::sleep_for(500ms);

	pen(255, 0, 0, 2, true); // Levantar el lápiz antes de moverse

	// Letras "ROBORESCUE"
	drawLetterR(x);
	x += 1.0;
	drawLetterO(x);
	x += 0.75;
	drawLetterB(x);
	x += 1.25;
	drawLetterO(x);
	x += 0.9;
	drawLetterR(x);
	x += 0.5;
	drawLetterE(x);
	x += 1.3;
	drawLetterS(x);
	x += 1.55;
	drawLetterC(x);
	x += 0.2;
	drawLetterU(x);
	x += 1.0;
	drawLetterE(x);

	rclcpp::sleep_for(2s);

	pen(255, 0, 0, 2, true);
	move_to(4.88, 3, node_, pub_);
	pen(0, 255, 0, 2, false);

	drawTriangleUP(side - 0.5, true);
	forward(0.5);
	drawHexagonFlat(side);
	forward(side);
	pen(0, 255, 0, 2, false);
	drawTriangleUP(side - 0.5, false);

	drawRectangleAboveHexagon(side, 0.70);

	RCLCPP_INFO(node_->get_logger(), "Escena completada.");
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<rclcpp::Node>("shape_drawer");
	Controller ctrl(node);
	ctrl.drawScene();
	rclcpp::shutdown();
	return 0;
}

// ================= FUNCIONES AUXILIARES =====================

double euclidean_distance(double x1, double y1, double x2, double y2)
{
	return std::hypot(x2 - x1, y2 - y1);
}

double normalize_angle(double angle)
{
	while (angle > M_PI)
		angle -= 2 * M_PI;
	while (angle < -M_PI)
		angle += 2 * M_PI;
	return angle;
}

void rotate_to(double target_angle, rclcpp::Node::SharedPtr node,
			   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
	geometry_msgs::msg::Twist cmd;
	rclcpp::Rate rate(50);

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);
		double error = normalize_angle(target_angle - pose.theta);
		if (std::abs(error) < 0.01)
			break;
		cmd.angular.z = (error > 0 ? 1.0 : -1.0) * std::min(1.0, std::abs(error));
		pub->publish(cmd);
		rate.sleep();
	}
	pub->publish(geometry_msgs::msg::Twist{});
	rclcpp::sleep_for(50ms);
}

void move_to(double x_goal, double y_goal,
			 rclcpp::Node::SharedPtr node,
			 rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub)
{
	geometry_msgs::msg::Twist cmd;
	rclcpp::Rate rate(50);

	while (rclcpp::ok())
	{
		rclcpp::spin_some(node);

		double dx = x_goal - pose.x;
		double dy = y_goal - pose.y;
		double distance = std::hypot(dx, dy);

		if (distance < 0.01)
			break;

		double target_angle = std::atan2(dy, dx);
		double angle_error = normalize_angle(target_angle - pose.theta);

		cmd.linear.x = std::min(1.0, distance);
		cmd.angular.z = std::min(1.0, std::max(-1.0, angle_error));

		pub->publish(cmd);
		rate.sleep();
	}

	pub->publish(geometry_msgs::msg::Twist{});
	rclcpp::sleep_for(50ms);
}
