#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <allegro_hand_interfaces/action/move_fingertip_poses.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveFingertipPosesActionClient : public rclcpp::Node
{
public:
    using MoveFingertipPoses = allegro_hand_interfaces::action::MoveFingertipPoses;
    using GoalHandleMoveFingertipPoses = rclcpp_action::ClientGoalHandle<MoveFingertipPoses>;

    explicit MoveFingertipPosesActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<double>& coordinates,
                                    const std::vector<double>& angles,
                                    double percentage_vel)
    : Node("move_fingertip_poses_action_client", options),
      coordinates_(coordinates),
      angles_(angles),
      percentage_velocity_(percentage_vel)
    {
    this->client_ptr_ = rclcpp_action::create_client<MoveFingertipPoses>(
        this,
        "/move_fingertip_poses");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveFingertipPosesActionClient::send_goal_move_fingertip_poses, this));
    }

    void send_goal_move_fingertip_poses()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveFingertipPoses::Goal();

        if (coordinates_.size() % 3 != 0) {
            throw std::runtime_error("The size of the input vector must be a multiple of 3.");
        }

        for (size_t i = 0; i < coordinates_.size(); i += 3) {
            geometry_msgs::msg::Point point;
            point.x = coordinates_[i];
            point.y = coordinates_[i + 1];
            point.z = coordinates_[i + 2];
            coordinates_msg.push_back(point);
        }
        goal_msg.coordinates =  coordinates_msg;
        goal_msg.euler_angles =  angles_;
        goal_msg.percentage_velocity = percentage_velocity_;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveFingertipPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveFingertipPosesActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveFingertipPosesActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveFingertipPosesActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveFingertipPoses>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> coordinates_, angles_;
    std::vector<geometry_msgs::msg::Point> coordinates_msg;
    double percentage_velocity_;

    void goal_response_callback(const GoalHandleMoveFingertipPoses::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleMoveFingertipPoses::SharedPtr,
    const std::shared_ptr<const MoveFingertipPoses::Feedback> feedback)
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

    void result_callback(const GoalHandleMoveFingertipPoses::WrappedResult & result)
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
            ss << "The move_fingertip_poses action has been completed succesfully: ";
        } else{
            ss << "The move_fingertip_poses action has failed ";
        }
        
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::vector<double> fingertip_coordinates, fingertip_orientations;
    double percentage_velocity;

    // If not enough arguments we use the predefined ones
    if (argc < 27) {
        std::cerr << "Usage: " << argv[0] << " <12 fingertips coordinates values> <12 fingertips orientation values> <percentage_velocity>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;

        fingertip_orientations = {
            0.311448, 1.28499, 0.085943, 
            3.1336, -0.922871, -3.14159, 
            2.8249, -1.24865, 3.05492, 
            3.02162, -0.815196, -1.19429
        };

        fingertip_coordinates = {
            0.0692146, -0.0276369, 0.0931639, 
            0.0728771, 0, 0.111519, 
            0.0546154, 0.032539, 0.0969059, 
            0.0737254, -0.101786, -0.0223788
        };
        
        percentage_velocity = 0.2;
        // return 1;
    }
    else{
        for (int i = 1; i <= 12; ++i) {
            fingertip_coordinates.push_back(std::stod(argv[i]));
        }
         for (int i = 13; i <= 25; ++i) {
            fingertip_orientations.push_back(std::stod(argv[i]));
        }
        percentage_velocity = std::stod(argv[26]);
    }

    auto move_fingertip_poses_client_node = std::make_shared<MoveFingertipPosesActionClient>(rclcpp::NodeOptions(),fingertip_coordinates, fingertip_orientations, percentage_velocity);

    rclcpp::spin(move_fingertip_poses_client_node);
            
    return 0;
}