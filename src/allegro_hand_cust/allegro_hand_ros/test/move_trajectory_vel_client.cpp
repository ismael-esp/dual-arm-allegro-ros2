#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <allegro_hand_interfaces/action/move_trajectory_vel.hpp>
#include "allegro_hand/AllegroHand.h"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveTrajectoryVelActionClient : public rclcpp::Node
{
public:
    using MoveTrajectoryVel = allegro_hand_interfaces::action::MoveTrajectoryVel;
    using GoalHandleMoveTrajectory = rclcpp_action::ClientGoalHandle<MoveTrajectoryVel>;

    explicit MoveTrajectoryVelActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<AllegroHandDriver::Vector>& trajectory,
                                    double percentage_vel)
    : Node("move_trajectory_vel_action_client", options),
      trajectory_(trajectory),
      max_velocity_(percentage_vel)
    {
    this->client_ptr_ = rclcpp_action::create_client<MoveTrajectoryVel>(
        this,
        "/move_trajectory_vel");
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&MoveTrajectoryVelActionClient::send_goal_move_trajectory_vel, this));
    }

    void send_goal_move_trajectory_vel()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = MoveTrajectoryVel::Goal();
        goal_msg.trajectory.resize(trajectory_.size());
        for (long unsigned int i=0; i < trajectory_.size(); i++){
            goal_msg.trajectory[i].hand_configuration.resize(16);
            for (int ii=0; ii < 16; ii ++){
                goal_msg.trajectory[i].hand_configuration[ii] = trajectory_[i](ii);
            }
        }
        goal_msg.max_velocity = max_velocity_;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<MoveTrajectoryVel>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MoveTrajectoryVelActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MoveTrajectoryVelActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MoveTrajectoryVelActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<MoveTrajectoryVel>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<AllegroHandDriver::Vector> trajectory_;
    double max_velocity_;

    void goal_response_callback(const GoalHandleMoveTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleMoveTrajectory::SharedPtr,
    const std::shared_ptr<const MoveTrajectoryVel::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Current configuration: " << feedback->current_configuration.header.frame_id << ". Position: ";
        for (auto number : feedback->current_configuration.position) {
            ss << number << " ";
        }
        ss << "\n Desired configuration: " << feedback->desired_current_configuration.header.frame_id << ". Position: ";
        for (auto number : feedback->desired_current_configuration.position) {
            ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    }

    void result_callback(const GoalHandleMoveTrajectory::WrappedResult & result)
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
            ss << "The move_trajectory_vel action has been completed succesfully: ";
        } else{
            ss << "The move_trajectory_vel action has failed ";
        }
        
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::vector<AllegroHandDriver::Vector> trajectory_values_;
    std::string file_name;
    AllegroHand* allegro_hand_read_trajectory = new AllegroHand(true, "V4R284");
    double max_velocity;
    // If not enough arguments we use the predefined ones
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <trajectory name> <maximum_velocity[rad/s]>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;
        file_name = "prismatic_3";
        max_velocity = 1;  // 1 rad/s
    }
    else{
        file_name = (argv[1]);
        max_velocity = std::stod(argv[2]);
    }
    allegro_hand_read_trajectory->readTrajectory(file_name, trajectory_values_);
    std::vector<AllegroHandDriver::Vector> trajectory_values = trajectory_values_; 
    auto move_trajectory_vel_client_node = std::make_shared<MoveTrajectoryVelActionClient>(rclcpp::NodeOptions(), trajectory_values, max_velocity);

    rclcpp::spin(move_trajectory_vel_client_node);
    // delete allegro_hand_read_trajectory;

    return 0;
}