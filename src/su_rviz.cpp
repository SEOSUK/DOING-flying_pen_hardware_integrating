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
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include "test_pkg/su_rot.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;


class su_rviz : public rclcpp::Node {
public:
    su_rviz() : Node("su_rviz"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
   {
      rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10))
                                      .reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE)
                                      .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);


        //PUBLISHER GROUP
        cf_vel_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/cf_1/velocity", 10);
        cf_EE_vel_arrow_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
          "/rviz/cf_1_EE/velocity", 10);
  

          
        //SUBSCRIBER GROUP
        cf_pose_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/xyzrpy", qos_settings,  // Topic name and QoS depth
          std::bind(&su_rviz::cf_pose_subscriber, this, std::placeholders::_1));

        cf_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/xyzrpy_vel", qos_settings,
          std::bind(&su_rviz::cf_velocity_subscriber, this, std::placeholders::_1));
  
        global_EE_xyz_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_xyzrpy", qos_settings,
          std::bind(&su_rviz::global_EE_xyz_callback, this, std::placeholders::_1));

        global_EE_xyz_vel_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/pen/EE_xyzrpy_vel", qos_settings,
          std::bind(&su_rviz::global_EE_xyz_vel_callback, this, std::placeholders::_1));


        timer_ = this->create_wall_timer(
          10ms, std::bind(&su_rviz::visual_timer_callback, this));

      }


    private:
      void visual_timer_callback()
      {
        cf_EE_pose_FK_tf_publisher();   // End Effector postion 시각화
        cf_EE_vel_FK_arrow_publisher();


        cf_pose_tf_publisher();   //Crazyflie position 시각화
        cf_vel_arrow_publisher();    // crazyflie velocity 시각화
      }


    void cf_pose_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      global_xyz_meas[0] = msg->data[0];
      global_xyz_meas[1] = msg->data[1];
      global_xyz_meas[2] = msg->data[2];

      body_rpy_meas[0] = msg->data[3];
      body_rpy_meas[1] = msg->data[4];
      body_rpy_meas[2] = msg->data[5];

      //body rpy meas 0 1 2 를 기반으로 R_B 생성
      Eigen::Matrix3d Rz, Ry, Rx;

      Rz << cos(body_rpy_meas[2]), -sin(body_rpy_meas[2]), 0,
            sin(body_rpy_meas[2]),  cos(body_rpy_meas[2]), 0,
                 0  ,       0  , 1;
  
      Ry << cos(body_rpy_meas[1]), 0, sin(body_rpy_meas[1]),
                   0 , 1,     0,
           -sin(body_rpy_meas[1]), 0, cos(body_rpy_meas[1]);
  
      Rx << 1,      0       ,       0,
            0, cos(body_rpy_meas[0]), -sin(body_rpy_meas[0]),
            0, sin(body_rpy_meas[0]),  cos(body_rpy_meas[0]);
  
      R_B = Rz * Ry * Rx;
    }

    void cf_velocity_subscriber(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      global_xyz_vel_meas[0] = msg->data[0];
      global_xyz_vel_meas[1] = msg->data[1];
      global_xyz_vel_meas[2] = msg->data[2];

      body_omega_meas[0] = msg->data[3];
      body_omega_meas[1] = msg->data[4];
      body_omega_meas[2] = msg->data[5];
    }

    void global_EE_xyz_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_EE_xyz_meas[0] = msg->data[0];
      global_EE_xyz_meas[1] = msg->data[1];
      global_EE_xyz_meas[2] = msg->data[2];
      global_EE_rpy_meas[0] = msg->data[3];
      global_EE_rpy_meas[1] = msg->data[4];
      global_EE_rpy_meas[2] = msg->data[5];
    }

    void global_EE_xyz_vel_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg){
      global_EE_xyz_vel_meas[0] = msg->data[0];
      global_EE_xyz_vel_meas[1] = msg->data[1];
      global_EE_xyz_vel_meas[2] = msg->data[2];

      body_omega_meas[0] = msg->data[3];
      body_omega_meas[1] = msg->data[4];
      body_omega_meas[2] = msg->data[5];
    }

    void cf_pose_tf_publisher() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "Body_frame";

        transformStamped.transform.translation.x = global_xyz_meas[0];
        transformStamped.transform.translation.y = global_xyz_meas[1];
        transformStamped.transform.translation.z = global_xyz_meas[2];

        tf2::Quaternion quat;
        quat.setRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);
        std::cout << "body_rpy" << body_rpy_meas << std::endl;
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transformStamped);

    }

    void cf_vel_arrow_publisher(){
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "body_vel";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start_point, end_point;
    start_point.x = global_xyz_meas[0]; 
    start_point.y = global_xyz_meas[1];
    start_point.z = global_xyz_meas[2];

    end_point.x = start_point.x + global_xyz_vel_meas[0];
    end_point.y = start_point.y + global_xyz_vel_meas[1];
    end_point.z = start_point.z + global_xyz_vel_meas[2];

    marker.points.push_back(start_point);
    marker.points.push_back(end_point);


    marker.scale.x = 0.02; 
    marker.scale.y = 0.05; 
    marker.scale.z = 0.05;

    marker.color.a = 1.0; 
    marker.color.r = 1.0; 
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    cf_vel_arrow_publisher_->publish(marker);
    }
    
    void cf_EE_vel_FK_arrow_publisher(){
      
      auto marker = visualization_msgs::msg::Marker();
      marker.header.frame_id = "world";
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "EE_vel";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
  
      geometry_msgs::msg::Point start_point, end_point;
      start_point.x = global_EE_xyz_meas[0]; 
      start_point.y = global_EE_xyz_meas[1];
      start_point.z = global_EE_xyz_meas[2];
  
      end_point.x = start_point.x + global_EE_xyz_vel_meas[0];
      end_point.y = start_point.y + global_EE_xyz_vel_meas[1];
      end_point.z = start_point.z + global_EE_xyz_vel_meas[2];
  
      marker.points.push_back(start_point);
      marker.points.push_back(end_point);
  
  
      marker.scale.x = 0.02; 
      marker.scale.y = 0.05; 
      marker.scale.z = 0.05;
  
      marker.color.a = 1.0; 
      marker.color.r = 1.0; 
      marker.color.g = 0.0;
      marker.color.b = 0.0;
  
      cf_EE_vel_arrow_publisher_->publish(marker);
      }


    void cf_EE_pose_FK_tf_publisher()
    {
        geometry_msgs::msg::TransformStamped transform_msg;
        transform_msg.header.stamp = this->get_clock()->now();
        transform_msg.header.frame_id = "world";  // 부모 프레임
        transform_msg.child_frame_id = "EE_frame"; // 자식 프레임 (EE)

        transform_msg.transform.translation.x = global_EE_xyz_meas[0];
        transform_msg.transform.translation.y = global_EE_xyz_meas[1];
        transform_msg.transform.translation.z = global_EE_xyz_meas[2];

        tf2::Quaternion quat;
        quat.setRPY(body_rpy_meas[0], body_rpy_meas[1], body_rpy_meas[2]);

        transform_msg.transform.rotation.x = quat.x();
        transform_msg.transform.rotation.y = quat.y();
        transform_msg.transform.rotation.z = quat.z();
        transform_msg.transform.rotation.w = quat.w();

        // TF Broadcast
        tf_broadcaster_->sendTransform(transform_msg);
    }    

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_vel_arrow_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cf_EE_vel_arrow_publisher_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_pose_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cf_vel_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr global_EE_xyz_vel_subscriber_;


    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;


    Eigen::Vector3d global_xyz_meas;
    Eigen::Vector3d body_rpy_meas;
    Eigen::Vector3d global_rpy_meas;
    Eigen::Vector3d body_omega_meas;

    Eigen::Vector3d global_xyz_vel_meas;
    Eigen::Vector3d body_xyz_vel_meas;
    Eigen::Matrix3d R_B;

    Eigen::Vector3d global_EE_xyz_meas;
    Eigen::Vector3d global_EE_rpy_meas;
    Eigen::Vector3d global_EE_xyz_vel_meas;

};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<su_rviz>());
    rclcpp::shutdown();
    return 0;
}