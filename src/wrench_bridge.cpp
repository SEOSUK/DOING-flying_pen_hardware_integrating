#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <chrono>

#include <gz/transport/Node.hh>
#include <gz/msgs/wrench.pb.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class GzToRosWrenchBridge : public rclcpp::Node
{
public:
  GzToRosWrenchBridge() : Node("gz_to_ros_wrench_bridge"),
  force_filter(3, 0.7, 0.03)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Wrench>("/ee/force_wrench", 10);

    // Subscribe to Gazebo Transport topic
    node_.Subscribe("/gz/EE_forcetorque", &GzToRosWrenchBridge::gzCallback, this);

    RCLCPP_INFO(this->get_logger(), "Subscribed to /gz/EE_forcetorque, publishing to /ee/force_wrench");

    timer_ = this->create_wall_timer(
      10ms, std::bind(&GzToRosWrenchBridge::timer_callback, this));


    numerical_timer_ = this->create_wall_timer(
      30ms, std::bind(&GzToRosWrenchBridge::numerical_timer_callback, this));
      
  }

private:

void numerical_timer_callback()
{
  force_lpf();

}

void timer_callback()
{
  data_publish();  
}


  void force_lpf()
  {
    wrench_meas_lpf = force_filter.apply(wrench_meas);
  }

  void data_publish()
  {
    geometry_msgs::msg::Wrench wrench;

    wrench.force.x = wrench_meas_lpf[0];
    wrench.force.y = wrench_meas_lpf[1];
    wrench.force.z = wrench_meas_lpf[2];

    wrench.torque.x = wrench_meas_lpf[3];
    wrench.torque.y = wrench_meas_lpf[4];
    wrench.torque.z = wrench_meas_lpf[5];

    publisher_->publish(wrench);
  }


  void gzCallback(const gz::msgs::Wrench &msg)
  {
    geometry_msgs::msg::Wrench wrench;

    wrench_meas[0] = msg.force().x();
    wrench_meas[1] = msg.force().y();
    wrench_meas[2] = msg.force().z();

    wrench_meas[3] = msg.torque().x();
    wrench_meas[4] = msg.torque().y();
    wrench_meas[5] = msg.torque().z();

    publisher_->publish(wrench);
  }


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr numerical_timer_;

  FilteredVector force_filter;


  Eigen::VectorXd wrench_meas = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd wrench_meas_lpf = Eigen::VectorXd::Zero(6);

  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr publisher_;
  gz::transport::Node node_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzToRosWrenchBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
