#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <allegro_hand_interfaces/action/move_config_finger.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveConfigFingerActionClient : public rclcpp::Node
{
public:
    using MoveConfigFinger = allegro_hand_interfaces::action::MoveConfigFinger;
    using GoalHandleMoveConfigFinger = rclcpp_action::ClientGoalHandle<MoveConfigFinger>;

    explicit MoveConfigFingerActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<double>& final_config,
                                    int finger_id,
                                    double percentage_vel)
    : Node("move_config_finger_action_client", options),
      final_configuration_(final_config),
      percentage_velocity_(percentage_vel),
      finger_id_(finger_id)
    {
    this->client_ptr_ = rclcpp_action::create_client<MoveConfigFinger>(
        this,
        "/move_conf_finger");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveConfigFingerActionClient::send_goal_move_config, this));
    }

    void send_goal_move_config()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveConfigFinger::Goal();
        goal_msg.final_configuration =  final_configuration_;
        goal_msg.percentage_velocity = percentage_velocity_;
        goal_msg.finger_id = finger_id_;
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveConfigFinger>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveConfigFingerActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveConfigFingerActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveConfigFingerActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveConfigFinger>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> final_configuration_;
    double percentage_velocity_;
    int finger_id_;

    void goal_response_callback(const GoalHandleMoveConfigFinger::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleMoveConfigFinger::SharedPtr,
    const std::shared_ptr<const MoveConfigFinger::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Configuration: " << feedback->current_configuration.header.frame_id << ". Position: ";
        for (auto number : feedback->current_configuration.position) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleMoveConfigFinger::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
            break;
            case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
            case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
            default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        std::stringstream ss;
        if(result.result->status){
            ss << "The move_config_finger action has been completed succesfully: ";
        } else{
            ss << "The move_config_finger action has failed ";
        }
        
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::vector<double> final_configuration;
    double percentage_velocity;
    int finger_id;

    // If not enough arguments we use the predefined ones
    if (argc < 7) {
        std::cerr << "Usage: " << argv[0] << " <4 finger final_configuration values> <percentage_velocity> <finger_id {0,1,2,3}>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;
        final_configuration = {-0.35780429446246664, 0.036747947880789181, 0.82771645890907997, 0.4559763484628358};
        percentage_velocity = 0.50;
        finger_id = 0;
        // return 1;
    }
    else{
        for (int i = 1; i <= 4; ++i) {
            final_configuration.push_back(std::stod(argv[i]));
        }
        percentage_velocity = std::stod(argv[5]);
        finger_id = std::stod(argv[6]);
    }

    auto move_config_finger_client_node = std::make_shared<MoveConfigFingerActionClient>(rclcpp::NodeOptions(), final_configuration, finger_id, percentage_velocity);

    rclcpp::spin(move_config_finger_client_node);
            
    return 0;
}