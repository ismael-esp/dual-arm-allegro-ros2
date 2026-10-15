#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <allegro_hand_interfaces/action/move_config.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveConfigActionClient : public rclcpp::Node
{
public:
    using MoveConfig = allegro_hand_interfaces::action::MoveConfig;
    using GoalHandleMoveConfig = rclcpp_action::ClientGoalHandle<MoveConfig>;

    explicit MoveConfigActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<double>& final_config,
                                    double percentage_vel)
    : Node("move_config_action_client", options),
      final_configuration_(final_config),
      percentage_velocity_(percentage_vel)
    {
    this->client_ptr_ = rclcpp_action::create_client<MoveConfig>(
        this,
        "/move_conf");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveConfigActionClient::send_goal_move_config, this));
    }

    void send_goal_move_config()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveConfig::Goal();
        goal_msg.final_configuration =  final_configuration_;
        goal_msg.percentage_velocity = percentage_velocity_;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveConfig>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveConfigActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveConfigActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveConfigActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveConfig>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> final_configuration_;
    double percentage_velocity_;

    void goal_response_callback(const GoalHandleMoveConfig::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleMoveConfig::SharedPtr,
    const std::shared_ptr<const MoveConfig::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Configuration: " << feedback->current_configuration.header.frame_id << ". Position: ";
        for (auto number : feedback->current_configuration.position) {
            ss << number << " ";
        }
        // Alternative if feedback_current_configuration is a std::vector
        // ss << " Current configuration: ";
        // for (auto number : feedback->current_configuration) {
        //     ss << number << " ";
        // }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleMoveConfig::WrappedResult & result)
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
            ss << "The move_config action has been completed succesfully: ";
        } else{
            ss << "The move_config action has failed ";
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

    // If not enough arguments we use the predefined ones
    if (argc < 18) {
        std::cerr << "Usage: " << argv[0] << " <16 final_configuration values> <percentage_velocity>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;
        final_configuration = {-0.35780429446246664, 0.036747947880789181, 0.82771645890907997, 0.4559763484628358, 0.025652552989246551, 0.10181134352479515, 0.66847535142566017, 0.16509947598615426, 0.33596855731591074, -0.17433084453591777, 0.90485164419508435, 0.53728340222806015, 0.3259383203339562, 0.76771256333561744, 0.46263358539776134, 0.83419616952574083};
        percentage_velocity = 0.05;
        // return 1;
    }
    else{
        for (int i = 1; i <= 16; ++i) {
            final_configuration.push_back(std::stod(argv[i]));
        }
        percentage_velocity = std::stod(argv[17]);
    }

    auto move_config_client_node = std::make_shared<MoveConfigActionClient>(rclcpp::NodeOptions(), final_configuration, percentage_velocity);

    rclcpp::spin(move_config_client_node);
            
    return 0;
}