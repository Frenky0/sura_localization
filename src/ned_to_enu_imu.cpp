#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <string>

namespace
{

geometry_msgs::msg::Quaternion multiply_quaternions(
  const geometry_msgs::msg::Quaternion & left,
  const geometry_msgs::msg::Quaternion & right)
{
  geometry_msgs::msg::Quaternion result;
  result.w = left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z;
  result.x = left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y;
  result.y = left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x;
  result.z = left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w;
  return result;
}

geometry_msgs::msg::Quaternion normalize_quaternion(geometry_msgs::msg::Quaternion quaternion)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x +
    quaternion.y * quaternion.y +
    quaternion.z * quaternion.z +
    quaternion.w * quaternion.w);
  if (norm <= 0.0) {
    geometry_msgs::msg::Quaternion identity;
    identity.w = 1.0;
    return identity;
  }
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
  return quaternion;
}

geometry_msgs::msg::Quaternion rotate_orientation(
  const geometry_msgs::msg::Quaternion & orientation)
{
  geometry_msgs::msg::Quaternion ned_to_enu;
  ned_to_enu.x = std::sqrt(0.5);
  ned_to_enu.y = std::sqrt(0.5);
  ned_to_enu.z = 0.0;
  ned_to_enu.w = 0.0;
  return normalize_quaternion(multiply_quaternions(ned_to_enu, orientation));
}

std::array<double, 9> transform_covariance(const std::array<double, 9> & covariance)
{
  constexpr double transform[3][3] = {
    {0.0, 1.0, 0.0},
    {1.0, 0.0, 0.0},
    {0.0, 0.0, -1.0},
  };

  std::array<double, 9> converted{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      double value = 0.0;
      for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
          value += transform[row][i] * covariance[i * 3 + j] * transform[col][j];
        }
      }
      converted[row * 3 + col] = value;
    }
  }
  return converted;
}

class NedToEnuImu : public rclcpp::Node
{
public:
  NedToEnuImu()
  : Node("ned_to_enu_imu")
  {
    declare_parameter<std::string>("input_topic", "/cirtesub/sensors/imu");
    declare_parameter<std::string>("output_topic", "/cirtesub/sensors/imu_enu");
    declare_parameter<std::string>("frame_id", "cirtesub/IMU");

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(output_topic, 10);
    subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        on_imu(*msg);
      });

    RCLCPP_INFO(
      get_logger(),
      "Converting NED IMU %s to ENU %s",
      input_topic.c_str(),
      output_topic.c_str());
  }

private:
  void on_imu(const sensor_msgs::msg::Imu & msg)
  {
    sensor_msgs::msg::Imu converted;
    converted.header = msg.header;
    converted.header.frame_id = get_parameter("frame_id").as_string();
    converted.orientation = rotate_orientation(msg.orientation);
    converted.orientation_covariance = transform_covariance(msg.orientation_covariance);

    converted.angular_velocity.x = msg.angular_velocity.y;
    converted.angular_velocity.y = msg.angular_velocity.x;
    converted.angular_velocity.z = -msg.angular_velocity.z;
    converted.angular_velocity_covariance = transform_covariance(msg.angular_velocity_covariance);

    converted.linear_acceleration.x = msg.linear_acceleration.y;
    converted.linear_acceleration.y = msg.linear_acceleration.x;
    converted.linear_acceleration.z = -msg.linear_acceleration.z;
    converted.linear_acceleration_covariance = transform_covariance(msg.linear_acceleration_covariance);

    publisher_->publish(converted);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NedToEnuImu>());
  rclcpp::shutdown();
  return 0;
}
