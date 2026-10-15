#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <allegro_hand_interfaces/action/move_fingertip_pose.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveFingertipPoseActionClient : public rclcpp::Node
{
public:
    using MoveFingertipPose = allegro_hand_interfaces::action::MoveFingertipPose;
    using GoalHandleMoveFingertipPose = rclcpp_action::ClientGoalHandle<MoveFingertipPose>;

    explicit MoveFingertipPoseActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<double>& coordinates,
                                    const std::vector<double>& angles,
                                    int finger_id,
                                    double percentage_vel)
    : Node("move_fingertip_pose_action_client", options),
      coordinates_(coordinates),
      angles_(angles),
      percentage_velocity_(percentage_vel),
      finger_id_(finger_id)
    {
    this->client_ptr_ = rclcpp_action::create_client<MoveFingertipPose>(
        this,
        "/move_fingertip_pose");

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveFingertipPoseActionClient::send_goal_move_fingertip_pose, this));
    }

    void send_goal_move_fingertip_pose()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveFingertipPose::Goal();

        if (coordinates_.size() % 3 != 0) {
            throw std::runtime_error("The size of the input vector must be a multiple of 3.");
        }

        coordinates_msg.x = coordinates_[0];
        coordinates_msg.y = coordinates_[1];
        coordinates_msg.z = coordinates_[2];
        goal_msg.coordinates =  coordinates_msg;
        goal_msg.euler_angles =  angles_;
        goal_msg.percentage_velocity = percentage_velocity_;
        goal_msg.finger_id = finger_id_;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveFingertipPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveFingertipPoseActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveFingertipPoseActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveFingertipPoseActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveFingertipPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<double> coordinates_, angles_;
    geometry_msgs::msg::Point coordinates_msg;
    double percentage_velocity_;
    int finger_id_;


    void goal_response_callback(const GoalHandleMoveFingertipPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleMoveFingertipPose::SharedPtr,
    const std::shared_ptr<const MoveFingertipPose::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Configuration: " << feedback->current_configuration.header.frame_id << ". Position: ";
        for (auto number : feedback->current_configuration.position) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }

    void result_callback(const GoalHandleMoveFingertipPose::WrappedResult & result)
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
            ss << "The move_fingertip_pose action has been completed succesfully: ";
        } else{
            ss << "The move_fingertip_pose action has failed ";
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
    int finger_id;

    // If not enough arguments we use the predefined ones
    if (argc < 27) {
        std::cerr << "Usage: " << argv[0] << " <3 fingertips coordinates values> <3 fingertips orientation values> <percentage_velocity> <finger_id {0,1,2,3}>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;

        fingertip_orientations = {
            0.311448, 1.28499, 0.085943
        };

        fingertip_coordinates = {
            0.0692146, -0.0276369, 0.0931639
        };
        
        percentage_velocity = 0.50;
        finger_id = 0;

        // return 1;
    }
    else{
        for (int i = 1; i <= 3; ++i) {
            fingertip_coordinates.push_back(std::stod(argv[i]));
        }
         for (int i = 4; i <= 6; ++i) {
            fingertip_orientations.push_back(std::stod(argv[i]));
        }
        percentage_velocity = std::stod(argv[7]);
        finger_id = std::stod(argv[8]);

    }

    auto move_fingertip_pose_client_node = std::make_shared<MoveFingertipPoseActionClient>(rclcpp::NodeOptions(),fingertip_coordinates, fingertip_orientations, finger_id, percentage_velocity);

    rclcpp::spin(move_fingertip_pose_client_node);
            
    return 0;
}