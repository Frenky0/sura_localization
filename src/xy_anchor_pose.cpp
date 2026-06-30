#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class XyAnchorPose : public rclcpp::Node
{
public:
  XyAnchorPose()
  : Node("xy_anchor_pose")
  {
    declare_parameter<std::string>("output_topic", "sensors/xy_anchor/pose_enu");
    declare_parameter<std::string>("frame_id", "world_enu");
    declare_parameter<double>("publish_rate_hz", 10.0);

    declare_parameter<double>("x", 0.0);
    declare_parameter<double>("y", 0.0);
    declare_parameter<double>("z", 0.0);

    declare_parameter<double>("xy_variance", 0.0001);
    declare_parameter<double>("z_variance", 99999.0);
    declare_parameter<double>("roll_variance", 99999.0);
    declare_parameter<double>("pitch_variance", 99999.0);
    declare_parameter<double>("yaw_variance", 99999.0);

    const auto output_topic = get_parameter("output_topic").as_string();
    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_topic, 10);

    double rate_hz = get_parameter("publish_rate_hz").as_double();
    if (rate_hz <= 0.0) {
      rate_hz = 10.0;
    }

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        publish_pose();
      });

    RCLCPP_INFO(
      get_logger(),
      "Publishing XY anchor pose on %s",
      output_topic.c_str());
  }

private:
  void publish_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;

    pose.header.stamp = now();
    pose.header.frame_id = get_parameter("frame_id").as_string();

    pose.pose.pose.position.x = get_parameter("x").as_double();
    pose.pose.pose.position.y = get_parameter("y").as_double();
    pose.pose.pose.position.z = get_parameter("z").as_double();

    pose.pose.pose.orientation.w = 1.0;

    pose.pose.covariance[0] = get_parameter("xy_variance").as_double();
    pose.pose.covariance[7] = get_parameter("xy_variance").as_double();
    pose.pose.covariance[14] = get_parameter("z_variance").as_double();
    pose.pose.covariance[21] = get_parameter("roll_variance").as_double();
    pose.pose.covariance[28] = get_parameter("pitch_variance").as_double();
    pose.pose.covariance[35] = get_parameter("yaw_variance").as_double();

    publisher_->publish(pose);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XyAnchorPose>());
  rclcpp::shutdown();
  return 0;
}