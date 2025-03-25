#include <rclcpp/rclcpp.hpp>
#include <crazyflie_interfaces/msg/hover.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>


class SuPositionCtrler : public rclcpp::Node
{
public:
  SuPositionCtrler()
  : Node("su_position_ctrler")
  {
    publisher_ = this->create_publisher<crazyflie_interfaces::msg::Hover>("/cf_1/cmd_hover", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 20Hz
      std::bind(&SuPositionCtrler::timer_callback, this)
    );

    //TODO : Gain
    P_gain.diagonal() << 1, 1, 1;
    I_gain.diagonal() << 0, 0, 0;
    D_gain.diagonal() << 0, 0, 0;


  }

private:
  void timer_callback()
  {
time_cnt ++;
time_real = time_cnt * sampling_time;
    pid_controller();
    data_publish();
  }


  void pid_controller()
  {
    // TODO : cmd_hover node는 vx, vy, yaw_dot, pos_z 를 받음
    // 즉, cmd_position에 도달하기 위한 cmd_vel 을 만들어야 하는것임
    // 따라서, cmd_vel = PID (cmd_pos - pos) 를 해야됨

    // error_pos = pos_cmd - pos_meas
    // error_pos_dot: 수치미분
    // error_pos_Integral: 수치적분
    // cmd_vel = P_gain * error_pos + I_gain * error_pos_Integral + D_gain * error_pos_dot 하면 깔쌈하게 나옴
    

    // 일단 지금은 대충 진행
    if (time_real < 10 && cmd_altitude < 0.5)
    cmd_altitude += 0.2 * 0.05; // 속도는 속도 변화량 곱하기 샘플링타임

    if (time_real > 10)
    cmd_altitude -= 0.4 * 0.05;
  }


  void data_publish()
  {

      auto msg = crazyflie_interfaces::msg::Hover();

      // ✅ 현재 시간 수동으로 stamp에 설정
      rclcpp::Time now = this->now();
      msg.header.stamp.sec = static_cast<int32_t>(now.seconds());
      msg.header.stamp.nanosec = now.nanoseconds() % 1000000000;

      msg.vx = cmd_vel[0];
      msg.vy = cmd_vel[1];
      msg.yaw_rate = cmd_vel[2];
      msg.z_distance = cmd_altitude;
      publisher_->publish(msg);
      
    RCLCPP_INFO(this->get_logger(), "%lf", time_real);
  }








  Eigen::VectorXd cmd_vel = Eigen::VectorXd::Zero(3); // vx, vy, yaw 임!!
  
  Eigen::Matrix3d P_gain;
  Eigen::Matrix3d I_gain;
  Eigen::Matrix3d D_gain;
  double time_cnt;
  double time_real;
  double cmd_altitude;
  double sampling_time = 0.01;

  rclcpp::Publisher<crazyflie_interfaces::msg::Hover>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SuPositionCtrler>());
  rclcpp::shutdown();
  return 0;
}
