#include <iostream>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>  // tf2::Quaternion 추가
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
#include "test_pkg/ButterworthFilter.hpp"
#include "test_pkg/FilteredVector.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>


using namespace std::chrono_literals;


class su_fkik : public rclcpp::Node {
public:
su_fkik() : Node("su_fkik"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
    body_rpy_meas_dot_filter(3, 10, 0.01) // LPF INIT
   {

    EE_offset_d << 0.08, 0, 0;


      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        /////////////////////
        //Publisher GRoup//
        /////////////////////
        global_EE_xyz_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/global_EE_xyz", qos_settings);            



        /////////////////////
        //SUBSCIRIBER GRoup//
        /////////////////////
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf_1/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_fkik::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf_1/velocity", qos_settings,
          std::bind(&su_fkik::cf_velocity_subscriber, this, std::placeholders::_1));



          control_loop_timer_ = this->create_wall_timer(
            10ms, std::bind(&su_fkik::control_loop_callback, this));

          numerical_calc_timer_ = this->create_wall_timer(
            10ms, std::bind(&su_fkik::numerical_calc_callback, this));
            
      }


    private:

      void numerical_calc_callback()
      {
        body_rpy_meas_dot_raw = (body_rpy_meas - body_rpy_meas_prev) / 0.01;
        body_rpy_meas_dot = body_rpy_meas_dot_filter.apply(body_rpy_meas_dot_raw);


        body_rpy_meas_prev = body_rpy_meas;
      }


      void control_loop_callback()
      {
        bool simulation_Flag = true;  // True 하면 simulation mode, false 하면 하드웨어 모드로 구상

        if(simulation_Flag) cf_sim_state_calc();


        cf_EE_vel_FK();
        cf_EE_position_FK();
        cf_EE_position_IK();
        if(simulation_Flag) cf_simulation_cmd_position_publisher();
        else cf_hardware_cmd_position_publisher();

        data_publish();
      }


      void data_publish()
      {
        std_msgs::msg::Float64MultiArray global_EE_xyz_msg;
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[0]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[1]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[2]);
        global_EE_xyz_publisher_->publish(global_EE_xyz_msg);


      }


      void cf_pose_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        global_xyz_meas[0] = msg->pose.position.x;
        global_xyz_meas[1] = msg->pose.position.y;
        global_xyz_meas[2] = msg->pose.position.z;
  
        tf2::Quaternion quat(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
  
        tf2::Matrix3x3 mat(quat);
        mat.getRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
  
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_B(i, j) = mat[i][j];
            }
        } // R_B matrix: 회전행렬, body frame에서 global frame으로 변환함
  
      }

    void cf_sim_state_calc()
    {
      // 현재로서 각속도 데이터를 direct feedback 받을 수 없으니 일단 수치적으로 각도->각속도 계산함
      double sin_roll = sin(body_rpy_meas[0]);
      double cos_roll = cos(body_rpy_meas[0]);

      double sin_pitch = sin(body_rpy_meas[1]);
      double cos_pitch = cos(body_rpy_meas[1]);



      omega_eulerRate_Mapping_matrix << 1, 0, -sin_pitch,
                                        0, cos_roll, sin_roll * cos_pitch,
                                        0, -sin_roll, cos_roll * sin_pitch;

      body_omega_meas = omega_eulerRate_Mapping_matrix * body_rpy_meas_dot;

      //TODO: 이거 계산값 제대로 나오는지 확인 해봐야 됨
    }

    void cf_velocity_subscriber(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) {
      body_xyz_vel_meas[0] = msg->values[0];
      body_xyz_vel_meas[1] = msg->values[1];
      body_xyz_vel_meas[2] = msg->values[2];

      global_xyz_vel_meas = R_B * body_xyz_vel_meas;
    }
    
    void cf_EE_position_FK() {
      //TODO: global_xyz_meas, body_rpy_meas 를 조합해서 end effector position data를 만들기
      global_EE_xyz_meas = global_xyz_meas + R_B * EE_offset_d;
    
      
    }  

    void cf_EE_vel_FK() {
      //TODO: global_xyz_vel_meas, body_rpy_meas, angular_velocity 를 조합해서 end effector velocity data를 만들기
      


    }

    void cf_EE_position_IK() {
      //TODO: End Effector poisition, End Effector yaw command를 받아서 drone position, drone yaw command로 변환하기.
  
    }
    
    void cf_simulation_cmd_position_publisher() {
      //TODO: crazyflie 시뮬레이션으로 직접 xyz position, yaw command 데이터를 publish 하는 곳.

    }    

    void cf_hardware_cmd_position_publisher() {
      //TODO: crazyflie 하드웨어쪽으로 직접 xyz position, yaw command input 데이터를 publish 하는 곳.

    }

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr numerical_calc_timer_;



    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_publisher_;



    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d body_rpy_meas_prev;
    Eigen::Vector3d body_rpy_meas_dot_raw;
    Eigen::Vector3d body_rpy_meas_dot;
    Eigen::Matrix3d omega_eulerRate_Mapping_matrix;
    Eigen::Vector3d body_omega_meas;
    Eigen::Vector3d global_rpy_meas;
    Eigen::Vector3d global_xyz_vel_meas;
    Eigen::Vector3d body_xyz_vel_meas;
    Eigen::Matrix3d R_B;

    Eigen::Vector3d global_EE_xyz_meas;
    Eigen::Vector3d EE_offset_d;

    FilteredVector body_rpy_meas_dot_filter;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_fkik>());
    rclcpp::shutdown();
    return 0;
}