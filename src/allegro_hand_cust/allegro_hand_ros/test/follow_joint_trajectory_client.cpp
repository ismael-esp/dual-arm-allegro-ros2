#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include "allegro_hand/AllegroHand.h"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class FollowJointTrajectoryActionClient : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    explicit FollowJointTrajectoryActionClient(const rclcpp::NodeOptions & options,
                                    const std::vector<AllegroHandDriver::Vector>& trajectory)
                                    // double percentage_vel)
    : Node("follow_joint_trajectory_action_client", options),
      trajectory_(trajectory)
    //   percentage_velocity_(percentage_vel)
    {
    this->client_ptr_ = rclcpp_action::create_client<FollowJointTrajectory>(
        this,
        "/follow_joint_trajectory");
    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&FollowJointTrajectoryActionClient::send_goal_follow_joint_trajectory, this));
    }

    void send_goal_follow_joint_trajectory()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.points.resize(trajectory_.size());
        for (long unsigned int i=0; i < trajectory_.size(); i++){
            goal_msg.trajectory.points[i].positions.resize(16);
            for (int ii=0; ii < 16; ii ++){
                goal_msg.trajectory.points[i].positions[ii] = trajectory_[i](ii);
            }
        }
        // Fill in the joint names of the feedback
        // Option fill it with predefined names
        for (char i = '0'; i <= '3'; ++i) {
            for (char j = '0'; j <= '3'; ++j) {
                goal_msg.trajectory.joint_names.push_back("j" + std::string(1, i) + std::string(1, j));
            }
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&FollowJointTrajectoryActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&FollowJointTrajectoryActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&FollowJointTrajectoryActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<AllegroHandDriver::Vector> trajectory_;
    // double percentage_velocity_;

    void goal_response_callback(const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
    GoalHandleFollowJointTrajectory::SharedPtr,
    const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Header: " << feedback->header.frame_id;
        ss << " Current configuration: ";
        for (int i=0; i<feedback->joint_names.size(); i++){
            ss << feedback->joint_names[i] << ": " << feedback->actual.positions[i] << ", ";
        }
        ss << "\n Desired configuration: ";
        for (int i=0; i<feedback->joint_names.size(); i++){
            ss << feedback->joint_names[i] << ": " << feedback->desired.positions[i] << ", ";
        }
        ss << "\n Error configuration: ";
        for (int i=0; i<feedback->joint_names.size(); i++){
            ss << feedback->joint_names[i] << ": " << feedback->error.positions[i] << ", ";
        }
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());

    }

    void result_callback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
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
        // std::stringstream ss;
        // if(result.result->status){
        //     ss << "The follow_joint_trajectory action has been completed succesfully: ";
        // } else{
        //     ss << "The follow_joint_trajectory action has failed ";
        // }
        
        // RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        rclcpp::shutdown();
    }
};  

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::vector<AllegroHandDriver::Vector> trajectory_values_;
    std::string file_name;
    AllegroHand* allegro_hand_read_trajectory = new AllegroHand(true, "V4R284");
    // If not enough arguments we use the predefined ones
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <trajectory name>" << std::endl;
        std::cerr << "NOT ENOUGH ARGUMENTS, USING PREDEFINED VALUES INSTEAD" << std::endl;
        file_name = "prismatic_3";
    }
    else{
        file_name = (argv[1]);
    }
    allegro_hand_read_trajectory->readTrajectory(file_name, trajectory_values_);
    std::vector<AllegroHandDriver::Vector> trajectory_values = trajectory_values_; 
    auto follow_joint_trajectory_client_node = std::make_shared<FollowJointTrajectoryActionClient>(rclcpp::NodeOptions(), trajectory_values);

    rclcpp::spin(follow_joint_trajectory_client_node);
    //delete allegro_hand_read_trajectory;

    return 0;
}