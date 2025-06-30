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

turtlesim::msg::Pose pose;

Controller::Controller(NodePtr node)
	: node_(node)
{
	using std::placeholders::_1;

	// Creamos el publicador de velocidad para la tortuga (/turtle1/cmd_vel)
	pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
		"/turtle1/cmd_vel", 10);

	// Creamos el subscriptor de pose para la tortuga (/turtle1/pose)
	sub_ = node_->create_subscription<turtlesim::msg::Pose>(
		"/turtle1/pose", 10, std::bind(&Controller::execute_command, this, _1));
}

void Controller::execute_command(const turtlesim::msg::Pose &msg)
{
	pose = msg;
}

void Controller::pen(uint8_t r, uint8_t g, uint8_t b,
					 uint8_t width, bool off)
{
	// Cliente al servicio /turtle1/set_pen para cambiar color/grosor
	auto client = node_->create_client<turtlesim::srv::SetPen>(
		"/turtle1/set_pen");
	// Espera hasta que el servicio esté disponible
	while (!client->wait_for_service(1s) && rclcpp::ok())
	{
		RCLCPP_INFO(node_->get_logger(),
					"Esperando /turtle1/set_pen...");
	}
	// Preparamos la petición
	auto req = std::make_shared<turtlesim::srv::SetPen::Request>();
	req->r = r;			// componente rojo
	req->g = g;			// componente verde
	req->b = b;			// componente azul
	req->width = width; // grosor
	req->off = off;		// si true, levanta el lápiz

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

		// Salir si el error es suficientemente pequeño
		if (std::abs(error) < 0.01)
			break;

		// Sentido de giro con velocidad constante o proporcional (simple)
		cmd.angular.z = (error > 0 ? 1.0 : -1.0) * angular_speed;
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

void Controller::drawHexagonFlat(double side)
{
	// Sincronizamos al eje X
	rotate_to(0, node_, pub_);
	// Hexágono con base horizontal
	for (int i = 0; i < 6; ++i)
	{
		forward(side);
		rotate_to(normalize_angle(pose.theta + M_PI / 3), node_, pub_);
	}
}

void Controller::drawTriangleUP(double side, bool left)
{
	// Sincronizamos al eje X
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
	// Sincronizamos al eje X
	//  Desde base del hexágono, estamos mirando al este
	rotate(M_PI / 2);  // mirar hacia arriba
	forward(height);   // subir
	rotate(-M_PI / 2); // mirar este
	forward(base);	   // base rectángulo
	rotate(-M_PI / 2); // mirar abajo
	forward(height);   // bajar
	rotate(-M_PI / 2); // mirar oeste
	forward(base);	   // volver
}

void Controller::drawCurves(double radius, double radians,
						std::string direction)
{
	double target_angle = radians;
	double linear = 1;
	double angular;
	// Definimos velocidades ya existentes
	if (direction == "up")      angular = linear / radius;
	else if (direction == "down")    angular = -linear / radius;
	else return;

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

		// Calculamos cuánto giró desde la última iteración
		double delta_theta = normalize_angle(pose.theta - last_theta);
		accumulated_angle += std::abs(delta_theta);
		last_theta = pose.theta;

		rate.sleep();
	}

	stop();
	rclcpp::sleep_for(50ms);
}



void Controller::drawScene()
{
	double side = 1.5;
	double letter_x_pose = 0.7;
	rclcpp::sleep_for(500ms);

	pen(255, 0, 0, 2, true); // desactivamos el lapiz
	move_to(letter_x_pose, 8.5, node_, pub_); //nos movemos a la posición inicial
	rotate_to(M_PI/2,node_, pub_);
	
	//R
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(2,1);
	rotate_to(M_PI/16,node_, pub_);
	drawCurves(0.5, M_PI, "down");
	rotate_to(-M_PI*3/8,node_, pub_);
	forward(1.1,1);
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 1;
	rotate_to(0,node_, pub_);
	move_to(letter_x_pose, 8.5, node_, pub_); //nos movemos a la posición de "o"
	rotate_to(0,node_, pub_);

	//o
	pen(0, 255, 0, 2, false); // lápiz verde
	drawCurves(0.5, 2*M_PI, "up");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 0.75;
	rotate_to(0,node_, pub_);
	move_to(letter_x_pose, 8.5, node_, pub_); //nos movemos a la posición de "b"
	rotate_to(M_PI/2,node_, pub_);

	//b
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(2,1);
	rotate_to(-M_PI/2,node_, pub_);
	forward(1,1);
	rotate_to(M_PI/16,node_, pub_);
	drawCurves(0.5, M_PI, "down");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 1.25;
	rotate_to(0,node_, pub_);
	move_to(letter_x_pose, 8.5, node_, pub_); //nos movemos a la posición de "o"
	rotate_to(0,node_, pub_);

	//o
	pen(0, 255, 0, 2, false); // lápiz verde
	drawCurves(0.5, 2*M_PI, "up");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 0.9;
	rotate_to(0,node_, pub_);
	move_to(letter_x_pose, 8.5, node_, pub_); //nos movemos a la posición de "R"
	rotate_to(M_PI/2,node_, pub_);

	//R
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(2,1);
	rotate_to(M_PI/16,node_, pub_);
	drawCurves(0.5, M_PI, "down");
	rotate_to(-M_PI*3/8,node_, pub_);
	forward(1.1,1);
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 0.5;
	rotate_to(M_PI/2,node_, pub_);
	move_to(letter_x_pose, 9, node_, pub_); //nos movemos a la posición de "e"
	rotate_to(0,node_, pub_);

	//e
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(0.85,1);
	rotate_to(M_PI/2,node_, pub_);
	drawCurves(0.5, M_PI*13/8, "up");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 1.3;
	rotate_to(-M_PI/8,node_, pub_);
	move_to(letter_x_pose, 8.55, node_, pub_); //nos movemos a la posición de "s"
	rotate_to(-M_PI/4,node_, pub_);

	//s
	pen(0, 255, 0, 2, false); // lápiz verde
	drawCurves(0.25, M_PI*8/8, "up");
	drawCurves(0.25, M_PI*9/8, "down");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 1.55;
	rotate_to(-M_PI/8,node_, pub_);
	move_to(letter_x_pose, 9, node_, pub_); //nos movemos a la posición de "c"
	rotate_to(M_PI/2,node_, pub_);
	drawCurves(0.5, M_PI/5, "up");

	//c
	pen(0, 255, 0, 2, false); // lápiz verde
	drawCurves(0.5, M_PI*11/8, "up");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 0.2;
	move_to(letter_x_pose, 9.5, node_, pub_); //nos movemos a la posición de "u"
	rotate_to(-M_PI/2,node_, pub_);

	//u
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(0.5,1);
	drawCurves(0.4, M_PI*7/8, "up");
	rotate_to(M_PI/2,node_, pub_);
	forward(0.45,1);
	pen(255, 0, 0, 2, true); // desactivamos el lapiz

	letter_x_pose = letter_x_pose + 1;
	rotate_to(-M_PI/3,node_, pub_);
	move_to(letter_x_pose, 9, node_, pub_); //nos movemos a la posición de "e"
	rotate_to(0,node_, pub_);

	//e
	pen(0, 255, 0, 2, false); // lápiz verde
	forward(0.85,1);
	rotate_to(M_PI/2,node_, pub_);
	drawCurves(0.5, M_PI*13/8, "up");
	pen(255, 0, 0, 2, true); // desactivamos el lapiz


rclcpp::sleep_for(5s);

	pen(255, 0, 0, 2, true); // desactivamos el lapiz
	move_to(3, 3, node_, pub_);
	pen(0, 255, 0, 2, false); // lápiz verde

	drawTriangleUP(side, true);

	drawHexagonFlat(side);
	forward(side);

	drawTriangleUP(side, false);

	//drawCurves(2.0, true, false, false, "right"); // círculo completo hacia la derecha
	//drawCurves(1.0, false, true, false, "up");	  // media circunferencia hacia arriba
	//drawCurves(1.0, false, false, true, "left");  // cuarto de circunferencia a la izquierda

	RCLCPP_INFO(node_->get_logger(), "Triángulo izquierdo terminado.");
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

// Funciones auxiliares
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

		// Calculo del error
		double dx = x_goal - pose.x;
		double dy = y_goal - pose.y;
		double distance = std::hypot(dx, dy); // distancia hasta el punto.

		// Calculamos cuánto falta para llegar al punto. Si distance < 0.01, ya hemos llegado.
		if (distance < 0.01)
			break;

		// CALCULO DEL ÁNGULO DESEADO
		// Usamos atan2 para obtener el angulo desde la tortuga hacia el punto objetivo
		// normalize_angle asegura que el error esté entre -pi y pi
		double target_angle = std::atan2(dy, dx);
		double angle_error = normalize_angle(target_angle - pose.theta);

		// PUBLICAMOS VELOCIDADES PARA MOVER Y CORREGIR
		//  Ajuste de dirección con algo de control proporcional
		cmd.linear.x = std::min(1.0, distance);						// velocidad proporcional
		cmd.angular.z = std::min(1.0, std::max(-1.0, angle_error)); // corrección de orientación

		pub->publish(cmd);
		rate.sleep();
	}

	pub->publish(geometry_msgs::msg::Twist{}); // stop
	rclcpp::sleep_for(50ms);
}
