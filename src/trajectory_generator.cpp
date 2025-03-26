#include <chrono>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <std_msgs/msg/float64_multi_array.hpp>  // 다중 float64 배열 퍼블리시
#include <std_msgs/msg/string.hpp>  // 다중 float64 배열 퍼블리시
#include <string> // std::string 헤더 추가
#include "std_msgs/msg/float64.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ncurses.h> // ncurses 헤더
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"

using namespace std::chrono_literals;

class trajectory_generator : public rclcpp::Node
{
public:
trajectory_generator()
  : Node("su_position_ctrler"),
  count_(0)
  {


      // QoS 설정
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    
    // TODO: git clone 받고 publisher 생성



    keyboard_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input", qos_settings,
      std::bind(&trajectory_generator::keyboard_subsciber_callback, this, std::placeholders::_1));

    global_EE_cmd_xyzYaw_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_cmd_xyzYaw", qos_settings);

      timer_ = this->create_wall_timer(
        10ms, std::bind(&trajectory_generator::timer_callback, this));
  

  }

private:
  void timer_callback()
  {
    time_cnt ++;
    time_real = time_cnt * sampling_time;

    if (time_real < 5) init_hovering(); // 붕 뜨기
    // external wrench observer를 여기에 하는게 나을지 fkik쪽에 넣는게 나을지 보류
    surface_trajectory_generation();

    data_publish();
  }


  void data_publish()
  {
    std_msgs::msg::Float64MultiArray global_xyz_EE_cmd_msg;
    global_xyz_EE_cmd_msg.data.push_back(global_cmd_xyzYaw[0]);
    global_xyz_EE_cmd_msg.data.push_back(global_cmd_xyzYaw[1]);
    global_xyz_EE_cmd_msg.data.push_back(global_cmd_xyzYaw[2]);
    global_xyz_EE_cmd_msg.data.push_back(global_cmd_xyzYaw[3]);
    global_EE_cmd_xyzYaw_publisher_->publish(global_xyz_EE_cmd_msg);


  }




  void init_hovering()
  {
    //TODO: 어제 잘 되던 hovering 코드 복붙사용

    if (time_real > 4.5 && time_real < 5) RCLCPP_INFO(this->get_logger(), "hovering done. ready to move");
  }


  void surface_trajectory_generation()
  {
  //  TODO: 법선 프레임 기반 회전행렬 R_C 정의하고, 변환 실시
  // 원래는 keyboard_subscriber_callback에서 contact_cmd_vel을 생성하게 두는게 낫겠지만, 일단 그냥 짜고 나중에 생각함
  //  global_cmd_vel = R_C * contact_cmd_vel;
  global_cmd_xyzYaw += global_cmd_vel_xyzYaw * sampling_time;

  }



  void keyboard_subsciber_callback(const std_msgs::msg::String::SharedPtr msg)
  {
      // 입력된 키를 문자열로 가져옴
      std::string input = msg->data;

        if (!input.empty()) // 입력 값이 비어있지 않을 경우
        {
            char input_char = input[0]; // 문자열의 첫 번째 문자만 사용

            if (input_char == 'w')
            {
              global_cmd_vel_xyzYaw[0] += 0.1;
            }
            else if (input_char == 's')
            {
              global_cmd_vel_xyzYaw[0] -= 0.1;
            }
            else if (input_char == 'a')
            {
              global_cmd_vel_xyzYaw[1] += 0.1;
            }
            else if (input_char == 'd')
            {
              global_cmd_vel_xyzYaw[1] -= 0.1;
            }
            else if (input_char == 'e')
            {
              global_cmd_vel_xyzYaw[2] += 0.1;
            }
            else if (input_char == 'q')
            {
              global_cmd_vel_xyzYaw[2] -= 0.1;
            }
            else if (input_char == 'z')
            {
              global_cmd_vel_xyzYaw[2] += 0.1;
            }
            else if (input_char == 'c')
            {
              global_cmd_vel_xyzYaw[2] -= 0.1;
            }
        }

    }


    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_cmd_xyzYaw_publisher_;


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_subscriber_; 





  size_t count_;
  Eigen::VectorXd global_cmd_xyzYaw = Eigen::Vector3d::Zero(4);
  Eigen::VectorXd global_cmd_vel_xyzYaw = Eigen::Vector3d::Zero(4);


  double time_cnt;
  double time_real;
  double sampling_time = 0.01;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<trajectory_generator>());
  rclcpp::shutdown();
  return 0;
}
