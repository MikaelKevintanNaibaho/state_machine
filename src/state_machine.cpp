#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "state_machine/move.hpp"
#include "state_machine/pwm_servo.hpp"
#include "state_machine/leg.hpp"

enum class RobotState
{
    Idle,
    Walk,
    TurnLeft,
    TurnRight
};

class StateMachineNode : public rclcpp::Node
{
public:
    StateMachineNode() : Node("state_machine_node"), current_state_(RobotState::Idle)
    {
        command_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_commands", 10, std::bind(&StateMachineNode::command_callback, this, std::placeholders::_1)
        );

        PCA9685_init();

        initialize_all_legs();

        // Set initial angles using forward kinematics
        stand_position();

        init_interrupt();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&StateMachineNode::state_machine_loop, this)
        );

    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    RobotState current_state_;

    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received command: %s", msg->data.c_str());

        if (msg->data == "MOVE_FORWARD")
        {
            current_state_ = RobotState::Walk;
        }
        else if (msg->data == "TURN_LEFT")
        {
            current_state_ = RobotState::TurnLeft;
        }
        else if (msg->data == "TURN_RIGHT")
        {
            current_state_ = RobotState::TurnRight;
        }
        else if (msg->data == "STOP")
        {
            current_state_ = RobotState::Idle;
        }
    }

    void state_machine_loop()
    {
        // Continuously check the current state
        while (rclcpp::ok()) {
            switch (current_state_)
            {
                case RobotState::Idle:
                    RCLCPP_INFO(this->get_logger(), "State: idle");
                    stand_position();
                    break;
                
                case RobotState::Walk:
                    RCLCPP_INFO(this->get_logger(), "State: Walk");
                    move_forward();
                    break;

                case RobotState::TurnLeft:
                    RCLCPP_INFO(this->get_logger(), "State: TurnLeft");
                    move_left_turn();
                    break;

                case RobotState::TurnRight:
                    RCLCPP_INFO(this->get_logger(), "State: TurnRight");
                    move_right_turn();
                    // Add implementation for turning right
                    break;

                default:
                    RCLCPP_WARN(this->get_logger(), "Unknown state!");
                    break;
            }

            // Sleep for a short duration to prevent busy-waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }



};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}
