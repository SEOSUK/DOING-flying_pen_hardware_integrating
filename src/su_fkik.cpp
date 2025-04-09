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
#include "crazyflie_interfaces/msg/position.hpp"


using namespace std::chrono_literals;


class su_fkik : public rclcpp::Node {
public:
su_fkik() : Node("su_fkik"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this)),
    body_rpy_meas_dot_filter(3, 1, 0.03), // LPF INIT
    body_omega_dot_filter(3, 1, 0.03), // LPF INIT
    global_xyz_meas_dot_raw_filter(3, 1, 0.03)    
   {

    EE_offset_d << 0.108, 0, 0; // MJ. EE_offset_d는 엔드이펙터의 길이 
    simulation_Flag = true;  // MJ. True 하면 simulation mode, false 하면 하드웨어 모드로 구상
    inverse_kinematics_Flag = true;
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

        /////////////////////
        //Publisher GRoup//
        /////////////////////
        global_xyz_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/xyzrpy", qos_settings);            
        global_xyz_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/xyzrpy_vel", qos_settings);            
        global_EE_xyz_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_xyzrpy", qos_settings);            
        global_EE_xyz_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/EE_xyzrpy_vel", qos_settings);            
        global_xyz_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pen/cmd_xyzYaw", qos_settings);            
        
        cf_global_xyzYaw_publisher_ = this->create_publisher<crazyflie_interfaces::msg::Position>("cf2/cmd_position", 10);



        test_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/test", qos_settings);            
        /////////////////////
        //SUBSCIRIBER GRoup//
        /////////////////////
        // SIMULATOR
        cf_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf2/pose", qos_settings,  // Topic name and QoS depth
          std::bind(&su_fkik::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf2/velocity", qos_settings,
          std::bind(&su_fkik::cf_velocity_subscriber, this, std::placeholders::_1));
        // HARDWARE
        cf_pose_real_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
          "/cf2/pose", qos_settings,
          std::bind(&su_fkik::cf_pose_real_subscriber, this, std::placeholders::_1));
                    
        cf_vel_real_subscriber_ = this->create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
          "/cf2/velocity", qos_settings,
          std::bind(&su_fkik::cf_velocity_real_subscriber, this, std::placeholders::_1));
        //end


        global_EE_cmd_xyzYaw_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_cmd_xyzYaw", qos_settings,
          std::bind(&su_fkik::global_EE_cmd_xyzYaw_subscriber, this, std::placeholders::_1));

          

          control_loop_timer_ = this->create_wall_timer(
            10ms, std::bind(&su_fkik::control_loop_callback, this));

          numerical_calc_timer_ = this->create_wall_timer(
            30ms, std::bind(&su_fkik::numerical_calc_callback, this));
            
      }


    private:

      void control_loop_callback()
      {
      // MJ state_calc: 필요한 센서 데이터를 추가적으로 가공
        if(simulation_Flag) cf_sim_state_calc();
        else cf_hardware_state_calc();


        cf_EE_vel_FK();
        cf_EE_position_FK();
        cf_EE_position_IK();

        data_publish();
      }

      void numerical_calc_callback()
      {
      //MJ 수치 미분/적분 sampling time이 민감하니 thread를 빼놨음. 시뮬과 real data는 따로 관리해야 하니 구분해놓음
        if(simulation_Flag) simulation_numerical_calc();
        else hardware_numerical_calc();        
      }
      
      void simulation_numerical_calc()
      {
          // MJ 현재 크레이지플라이 속도 데이터 및 각속도 데이터를 feedback 받을 수 없는 상황임.
          // 임시방편으로 위치/자세 데이터를 기반으로 수치적으로 처리 하겠음
          global_xyz_meas_dot_raw = (global_xyz_meas - global_xyz_meas_prev) / 0.03;
          global_xyz_vel_meas = global_xyz_meas_dot_raw_filter.apply(global_xyz_meas_dot_raw);
          global_xyz_meas_prev = global_xyz_meas;

          body_rpy_meas_dot_raw = (body_rpy_meas - body_rpy_meas_prev) / 0.03;
          body_rpy_meas_dot = body_rpy_meas_dot_filter.apply(body_rpy_meas_dot_raw);
          body_rpy_meas_prev = body_rpy_meas;
    
          body_omega_dot_raw = (body_omega_meas - body_omega_prev) / 0.03;
          body_alpha_meas = body_omega_dot_filter.apply(body_omega_dot_raw);
          body_omega_prev = body_omega_meas;
      }

      void hardware_numerical_calc()
      {
          // MJ 현재 크레이지플라이 속도 데이터 및 각속도 데이터를 feedback 받을 수 없는 상황임.
          // 임시방편으로 위치/자세 데이터를 기반으로 수치적으로 처리 하겠음
          global_xyz_meas_dot_raw = (global_xyz_meas - global_xyz_meas_prev) / 0.03;
          global_xyz_vel_meas = global_xyz_meas_dot_raw_filter.apply(global_xyz_meas_dot_raw);
          global_xyz_meas_prev = global_xyz_meas;

          body_rpy_meas_dot_raw = (body_rpy_meas - body_rpy_meas_prev) / 0.03;
          body_rpy_meas_dot = body_rpy_meas_dot_filter.apply(body_rpy_meas_dot_raw);
          body_rpy_meas_prev = body_rpy_meas;
    
          body_omega_dot_raw = (body_omega_meas - body_omega_prev) / 0.03;
          body_alpha_meas = body_omega_dot_filter.apply(body_omega_dot_raw);
          body_omega_prev = body_omega_meas;
      }

      void cf_sim_state_calc()
      {
        // 현재로서 각속도 데이터를 direct feedback 받을 수 없으니 일단 수치적으로 각도->각속도 계산함
        // 시뮬레이션만을 위한 코드임 일단은
        double sin_roll = sin(body_rpy_meas[0]);
        double cos_roll = cos(body_rpy_meas[0]);
  
        double sin_pitch = sin(body_rpy_meas[1]);
        double cos_pitch = cos(body_rpy_meas[1]);
  
        omega_eulerRate_Mapping_matrix << 1, sin_roll * tan(body_rpy_meas[1]), cos_roll * tan(body_rpy_meas[1]),
                                          0, cos_roll, -sin_roll,
                                        0, sin_roll / cos_pitch, cos_roll / cos_pitch;
  
        body_omega_meas = omega_eulerRate_Mapping_matrix * body_rpy_meas_dot;
        // std::cout << "body omega" << std::endl << body_omega_meas << std::endl;
        //TODO: 이거 계산값 제대로 나오는지 확인 해봐야 됨
      }
      
      void cf_hardware_state_calc()
      {
        // MJ. r p y 의 미분과 body coordinate 각속도의 변환 맵핑 매트릭스 구현
        // MJ. 자세 데이터를 기반으로 수치적으로 각속도 데이털르 구한 내용
        double sin_roll = sin(body_rpy_meas[0]);
        double cos_roll = cos(body_rpy_meas[0]);
  
        double sin_pitch = sin(body_rpy_meas[1]);
        double cos_pitch = cos(body_rpy_meas[1]);
  
        omega_eulerRate_Mapping_matrix << 1, sin_roll * tan(body_rpy_meas[1]), cos_roll * tan(body_rpy_meas[1]),
                                          0, cos_roll, -sin_roll,
                                        0, sin_roll / cos_pitch, cos_roll / cos_pitch;
  
        body_omega_meas = omega_eulerRate_Mapping_matrix * body_rpy_meas_dot;
      }

      void data_publish()
      {
        std_msgs::msg::Float64MultiArray global_xyz_msg;
        global_xyz_msg.data.push_back(global_xyz_meas[0]);
        global_xyz_msg.data.push_back(global_xyz_meas[1]);
        global_xyz_msg.data.push_back(global_xyz_meas[2]);
        global_xyz_msg.data.push_back(body_rpy_meas[0]);
        global_xyz_msg.data.push_back(body_rpy_meas[1]);
        global_xyz_msg.data.push_back(body_rpy_meas[2]);
        global_xyz_publisher_->publish(global_xyz_msg);

        std_msgs::msg::Float64MultiArray global_xyz_vel_msg;
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[0]);
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[1]);
        global_xyz_vel_msg.data.push_back(global_xyz_vel_meas[2]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[0]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[1]);
        global_xyz_vel_msg.data.push_back(body_omega_meas[2]);
        global_xyz_vel_publisher_->publish(global_xyz_vel_msg);        

        std_msgs::msg::Float64MultiArray global_EE_xyz_msg;
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[0]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[1]);
        global_EE_xyz_msg.data.push_back(global_EE_xyz_meas[2]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[0]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[1]);
        global_EE_xyz_msg.data.push_back(body_rpy_meas[2]);
        global_EE_xyz_publisher_->publish(global_EE_xyz_msg);

        crazyflie_interfaces::msg::Position global_xyzYaw_cmd_msg;
        global_xyzYaw_cmd_msg.x = global_xyz_cmd[0];
        global_xyzYaw_cmd_msg.y = global_xyz_cmd[1];
        global_xyzYaw_cmd_msg.z = global_xyz_cmd[2];
        global_xyzYaw_cmd_msg.yaw = global_EE_yaw_cmd * 180 / M_PI;
        cf_global_xyzYaw_publisher_->publish(global_xyzYaw_cmd_msg);


        std_msgs::msg::Float64MultiArray test_msg;
        test_msg.data.push_back(global_xyz_cmd[0]);
        test_msg.data.push_back(global_xyz_cmd[1]);
        test_msg.data.push_back(global_xyz_cmd[2]);
        test_msg.data.push_back(global_EE_yaw_cmd);
        test_msg.data.push_back(global_xyz_meas[0]);
        test_msg.data.push_back(global_xyz_meas[1]);
        test_msg.data.push_back(global_xyz_meas[2]);
        test_msg.data.push_back(body_rpy_meas[2]);
        test_publisher_->publish(test_msg);




        std_msgs::msg::Float64MultiArray global_EE_xyz_vel_msg;
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[0]);
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[1]);
        global_EE_xyz_vel_msg.data.push_back(global_EE_xyz_vel_meas[2]);
        global_EE_xyz_vel_publisher_->publish(global_EE_xyz_vel_msg);


        std_msgs::msg::Float64MultiArray global_xyz_cmd_msg;
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[0]);
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[1]);
        global_xyz_cmd_msg.data.push_back(global_xyz_cmd[2]);
        global_xyz_cmd_msg.data.push_back(global_EE_yaw_cmd);
        global_xyz_cmd_publisher_->publish(global_xyz_cmd_msg);
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
          double roll, pitch, yaw;
          mat.getRPY(roll, pitch, yaw);
      
          // Yaw 불연속 보정
          double delta_yaw = yaw - prev_yaw;
          if (delta_yaw > M_PI) {
              yaw_offset -= 2.0 * M_PI;  // -360도 보정
          } else if (delta_yaw < -M_PI) {
              yaw_offset += 2.0 * M_PI;  // +360도 보정
          }
          yaw_continuous = yaw + yaw_offset;  // 연속 yaw 업데이트
          prev_yaw = yaw;
      
          // RPY 업데이트
          body_rpy_meas[0] = roll;
          body_rpy_meas[1] = pitch;
          body_rpy_meas[2] = yaw_continuous;  // 보정된 Yaw 사용
      
          // Rotation matrix 업데이트
          for (int i = 0; i < 3; ++i) {
              for (int j = 0; j < 3; ++j) {
                  R_B(i, j) = mat[i][j];
              }
          }
      }
      
    void cf_velocity_subscriber(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg) {
      body_xyz_vel_meas[0] = msg->values[0];
      body_xyz_vel_meas[1] = msg->values[1];
      body_xyz_vel_meas[2] = msg->values[2];

      global_xyz_vel_meas = R_B * body_xyz_vel_meas;
    }
    
    void cf_pose_real_subscriber(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);

        // Yaw 불연속 보정
        double delta_yaw = yaw - prev_yaw;
        if (delta_yaw > M_PI) yaw_offset -= 2.0 * M_PI;
        else if (delta_yaw < -M_PI) yaw_offset += 2.0 * M_PI;

        yaw_continuous = yaw + yaw_offset;
        prev_yaw = yaw;

        body_rpy_meas[0] = roll;
        body_rpy_meas[1] = pitch;
        body_rpy_meas[2] = yaw_continuous;

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R_B(i, j) = mat[i][j];
    }

    void cf_velocity_real_subscriber(const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
    {
        body_xyz_vel_meas[0] = msg->values[0];
        body_xyz_vel_meas[1] = msg->values[1];
        body_xyz_vel_meas[2] = msg->values[2];

        global_xyz_vel_meas = R_B * body_xyz_vel_meas;
    }

    void global_EE_cmd_xyzYaw_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      global_EE_xyz_cmd[0] = msg->data[0];
      global_EE_xyz_cmd[1] = msg->data[1];
      global_EE_xyz_cmd[2] = msg->data[2];
      global_EE_yaw_cmd = msg->data[3];
    }

    void cf_EE_position_FK() {
      //TODO: global_xyz_meas, body_rpy_meas 를 조합해서 end effector position data를 만들기
      global_EE_xyz_meas = global_xyz_meas + R_B * EE_offset_d;
    
      
    }  

    void cf_EE_vel_FK() {
      //TODO: global_xyz_vel_meas, body_rpy_meas, angular_velocity 를 조합해서 end effector velocity data를 만들기
      global_EE_xyz_vel_meas = global_xyz_vel_meas + body_omega_meas.cross(R_B * EE_offset_d);


    }

    void cf_EE_position_IK() {
      //TODO: End Effector poisition, End Effector yaw command를 받아서 drone position, drone yaw command로 변환하기.

      R_B_cmd << cos(body_rpy_meas[2]), -sin(body_rpy_meas[2]), 0,
                sin(body_rpy_meas[2]),   cos(body_rpy_meas[2]), 0,
                0,                       0,                       1;


      if (inverse_kinematics_Flag) global_xyz_cmd = global_EE_xyz_cmd - R_B_cmd * EE_offset_d;
      else global_xyz_cmd = global_EE_xyz_cmd;
    
      // RCLCPP_INFO(this->get_logger(), "global_xyz_cmd [%lf] [%lf] [%lf] [%lf]", 
      // global_xyz_cmd[0], global_xyz_cmd[1], global_xyz_cmd[2], global_EE_yaw_cmd);
    }
  

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr numerical_calc_timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr test_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr global_xyz_cmd_publisher_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr cf_global_xyzYaw_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cf_pose_real_subscriber_;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr cf_vel_real_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_cmd_xyzYaw_subscriber_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d global_xyz_meas_dot_raw;
    Eigen::Vector3d global_xyz_meas_prev;
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d body_rpy_meas_prev;
    Eigen::Vector3d body_rpy_meas_dot_raw;
    Eigen::Vector3d body_rpy_meas_dot;

    Eigen::Vector3d body_rpy_meas_dot_prev;
    Eigen::Vector3d body_rpy_meas_ddot_raw;
    Eigen::Vector3d body_rpy_meas_ddot;

    Eigen::Matrix3d omega_eulerRate_Mapping_matrix;
    Eigen::Vector3d body_omega_meas;
    Eigen::Vector3d body_omega_prev;
    Eigen::Vector3d body_omega_dot_raw;
    Eigen::Vector3d body_alpha_meas;

    Eigen::Vector3d global_rpy_meas;
    Eigen::Vector3d global_xyz_vel_meas;
    Eigen::Vector3d body_xyz_vel_meas;
    Eigen::Matrix3d R_B;
    Eigen::Matrix3d R_B_cmd;


    Eigen::Vector3d global_EE_xyz_meas;
    Eigen::Vector3d global_EE_xyz_vel_meas;
    Eigen::Vector3d EE_offset_d;

    Eigen::Vector3d global_EE_xyz_cmd;
    Eigen::Vector3d global_xyz_cmd;
    double global_EE_yaw_cmd;
    double global_yaw_cmd;


    FilteredVector body_rpy_meas_dot_filter;
    FilteredVector body_omega_dot_filter;
    FilteredVector global_xyz_meas_dot_raw_filter;

    
    double prev_yaw, yaw_offset, yaw_continuous;
    bool simulation_Flag;
    bool inverse_kinematics_Flag;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_fkik>());
    rclcpp::shutdown();
    return 0;
}
