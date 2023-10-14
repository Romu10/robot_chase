#include "tf2/exceptions.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RickAndMortyController : public rclcpp::Node {
public:
  RickAndMortyController()
      : Node("rick_and_morty_controller"), error_distance_(1.0),
        error_yaw_(0.0),
        kp_distance_(0.2), // Ajusta este valor según tus necesidades
        kp_yaw_(0.1)       // Ajusta este valor según tus necesidades
  {
    // Inicializa el buffer y el oyente TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    // Publica el mensaje Twist
    publisher_ =
        create_publisher<geometry_msgs::msg::Twist>("/rick/cmd_vel", 10);

    // Define el temporizador para el lazo de control
    timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&RickAndMortyController::controlLoop, this));
  }

private:
  void controlLoop() {
    // Imprime un mensaje de advertencia
    RCLCPP_WARN(get_logger(), "Se encontró la transformación entre morty/base_link y rick/base_link");

    // Intenta obtener la transformación más reciente
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
          "morty/base_link", "rick/base_link", tf2::TimePoint());
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Error al obtener la transformación: %s",
                   ex.what());
      return;
    }

    // Calcula la distancia y el error angular
    double dx = transform.transform.translation.x;
    double dy = transform.transform.translation.y;
    error_distance_ = sqrt(dx * dx + dy * dy);
    RCLCPP_INFO(get_logger(), "Distancia: %f m", error_distance_);


    // Convierte la rotación de geometry_msgs::msg::Quaternion a tf2::Quaternion
    tf2::Quaternion rotation(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);

    // Calcula el yaw angle
    double yaw = rotation.getAngle();
    RCLCPP_INFO(get_logger(), "Yaw: %f", yaw);

    // Calcula la velocidad lineal y angular
    double linear_velocity = kp_distance_ * error_distance_;
    RCLCPP_INFO(get_logger(), "Linear Velocity: %f m", linear_velocity);
    double angular_velocity = kp_yaw_ * yaw;
    RCLCPP_INFO(get_logger(), "Angular Velocity: %f m", angular_velocity);

    // Crea y publica el mensaje Twist
    auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
    if (linear_velocity < 0.5){
        twist_msg->linear.x = linear_velocity;
    }
    else {
        twist_msg->linear.x = 0.15;
        RCLCPP_INFO(get_logger(), "Reducing Velocity To 0.1 For Security");
    }
    twist_msg->angular.z = angular_velocity;
    publisher_->publish(std::move(twist_msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  double error_distance_;
  double error_yaw_;
  double kp_distance_;
  double kp_yaw_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RickAndMortyController>());
  rclcpp::shutdown();
  return 0;
}
