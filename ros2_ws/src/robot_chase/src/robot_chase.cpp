#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class RobotChaseNode : public rclcpp::Node {
public:
  RobotChaseNode() : Node("robot_chase_node") {
    
    tf_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&RobotChaseNode::chase, this));
    
  }

private:
  void chase() {
    try {
      geometry_msgs::msg::TransformStamped transformStamped =
          tf_->lookupTransform("robot1/base_footprint", "robot2/base_footprint", tf2::TimePointZero);

      // Calculate distance and angular error
      float error_yaw = atan2(transformStamped.transform.translation.y, transformStamped.transform.translation.x);
      float error_distance = sqrt(pow(transformStamped.transform.translation.x, 2) + pow(transformStamped.transform.translation.y, 2));

      // Define gains
      double kp_distance = 0.5;
      double kp_yaw = 3;

      // Calculate linear and angular velocities
      double linear_velocity = kp_distance * error_distance;
      double angular_velocity = kp_yaw * error_yaw;

      // Create and publish Twist message
      geometry_msgs::msg::Twist twist;
      twist.linear.x = linear_velocity;
      twist.angular.z = angular_velocity;
      cmd_vel_publisher_->publish(twist);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Transform exception: %s", ex.what());
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tfl_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotChaseNode>());
  return 0;
}