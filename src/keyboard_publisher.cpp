#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ncurses.h> // ncurses 헤더
#include <string>

class keyboard_publisher : public rclcpp::Node
{
public:
  keyboard_publisher() : Node("command_generator")
    {
        // 퍼블리셔 생성
        publisher_ = this->create_publisher<std_msgs::msg::String>("/keyboard_input", 10);

        // ncurses 초기화
        initscr();           // ncurses 화면 초기화
        cbreak();            // 라인 버퍼링 비활성화
        noecho();            // 입력한 키 출력 방지
        nodelay(stdscr, TRUE); // 비블로킹 입력 모드
        keypad(stdscr, TRUE);

        RCLCPP_INFO(this->get_logger(), "키보드 퍼블리셔 노드가 시작되었습니다. 'q'를 눌러 종료하세요.");

        // 타이머를 사용해 주기적으로 키 입력 확인
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50), // 50ms마다 실행
            std::bind(&keyboard_publisher::checkKeyboardInput, this));
    }

    ~keyboard_publisher()
    {
        endwin(); // ncurses 종료
    }

private:
    void checkKeyboardInput()
    {
        int ch = getch(); // 키 입력 확인

        if (ch != ERR) // 입력이 있을 경우
        {
            char input_char = static_cast<char>(ch);

            if (input_char == 't') // 't' 입력 시 종료
            {
                RCLCPP_INFO(this->get_logger(), "종료 입력(t) 받음. 노드를 종료합니다.");
                rclcpp::shutdown();
                return;
            }

            // 메시지 생성 및 퍼블리시
            auto message = std_msgs::msg::String();
            message.data = std::string(1, input_char); // 입력 문자를 메시지로 변환
            publisher_->publish(message);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<keyboard_publisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
