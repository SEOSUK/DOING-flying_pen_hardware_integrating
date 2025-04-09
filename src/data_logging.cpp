#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <turtlesim/msg/pose.hpp>
#include <crazyflie_interfaces/msg/log_data_generic.hpp>

#include <rosidl_runtime_cpp/traits.hpp>
#include <unordered_set>
#include <chrono>
#include <iomanip>
#include <sstream>

// 현재 시간 문자열 생성 (예: "20250408_221045")
std::string get_time_string() {
  auto now = std::chrono::system_clock::now();
  std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

class RosbagRecorderNode : public rclcpp::Node {
public:
  RosbagRecorderNode() : Node("rosbag_cpp_recorder") {
    RCLCPP_INFO(this->get_logger(), "Starting rosbag recorder...");

    // 시간 기반 저장 경로
    std::string time_str = get_time_string();
    storage_options_.uri = "/home/mrl-seuk/sitl_crazy/CrazySim/ros2_ws/src/test_pkg/bag/my_bag_" + time_str;
    storage_options_.storage_id = "sqlite3";
    converter_options_.input_serialization_format = "cdr";
    converter_options_.output_serialization_format = "cdr";

    writer_.open(storage_options_, converter_options_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    
    
    // 각 토픽 구독 및 기록
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cf2/pose", qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        this->write_to_bag("/cf2/pose", *msg);
      });

    vel_sub_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
      "/cf2/velocity", qos,
      [this](const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) {
        this->write_to_bag("/cf2/velocity", *msg);
      });

    ee_cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/pen/EE_cmd_xyzYaw", qos,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        this->write_to_bag("/pen/EE_cmd_xyzYaw", *msg);
      });

    turtle_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", qos,
      [this](const turtlesim::msg::Pose::SharedPtr msg) {
        this->write_to_bag("/turtle1/pose", *msg);
      });

    turtle_cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", qos,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        this->write_to_bag("/turtle1/cmd_vel", *msg);
      });
  }

private:
  template <typename T>
  void write_to_bag(const std::string &topic_name, const T &msg) {
    // 토픽을 처음 쓰는 경우 등록
    if (created_topics_.find(topic_name) == created_topics_.end()) {
      writer_.create_topic({
        .name = topic_name,
        .type = rosidl_generator_traits::name<T>(),
        .serialization_format = "cdr",
        .offered_qos_profiles = ""
      });
      created_topics_.insert(topic_name);
    }

    // 메시지 직렬화 및 기록
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Serialization<T> serializer;
    serializer.serialize_message(&msg, &serialized_msg);

    writer_.write(serialized_msg, topic_name, "cdr", this->now());
  }

  rosbag2_cpp::Writer writer_;
  rosbag2_storage::StorageOptions storage_options_;
  rosbag2_cpp::ConverterOptions converter_options_;
  std::unordered_set<std::string> created_topics_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ee_cmd_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_vel_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RosbagRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
