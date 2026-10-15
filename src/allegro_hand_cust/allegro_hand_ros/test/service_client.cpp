#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "allegro_hand_interfaces/srv/get_controller.hpp"
#include "allegro_hand_interfaces/srv/get_gravity_vector.hpp"
#include "allegro_hand_interfaces/srv/get_hand_configuration.hpp"
#include "allegro_hand_interfaces/srv/get_pose_fk.hpp"
#include "allegro_hand_interfaces/srv/get_torque.hpp"
#include "allegro_hand_interfaces/srv/joint_limits.hpp"
#include "allegro_hand_interfaces/srv/coupled_closing.hpp"
#include "allegro_hand_interfaces/srv/set_controller.hpp"
#include "allegro_hand_interfaces/srv/set_des_pos.hpp"
#include "allegro_hand_interfaces/srv/set_des_pos_vel.hpp"
#include "allegro_hand_interfaces/srv/set_des_torque.hpp"
#include "allegro_hand_interfaces/srv/set_gravity_vector.hpp"
#include "allegro_hand_interfaces/srv/set_pose_ik.hpp"
#include "allegro_hand_interfaces/srv/stop_hand.hpp"
#include "allegro_hand_interfaces/srv/pause_finger.hpp"
#include "allegro_hand_interfaces/srv/continue_finger.hpp"
#include "allegro_hand_interfaces/srv/stop_finger.hpp"


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;


class ServiceClient : public rclcpp::Node
{
public:
    explicit ServiceClient(const std::string& prefix, const rclcpp::NodeOptions & options)
    : Node(prefix + "service_client", options),
     prefix_(prefix)
    {
        // getControllerSrvClient();
        // getGravityVectorSrvClient();
        // getHandConfigurationSrvClient();
        // getPoseFKSrvClient();
        // getTorqueSrvClient();
        // getJointLimitsSrvClient();
        // setControllerSrvClient("JointPositionPID");

        // std::vector<double> final_configuration = {-0.35780429446246664, 0.036747947880789181, 0.82771645890907997, 0.4559763484628358, 0.025652552989246551, 0.10181134352479515, 0.66847535142566017, 0.16509947598615426, 0.33596855731591074, -0.17433084453591777, 0.90485164419508435, 0.53728340222806015, 0.3259383203339562, 0.76771256333561744, 0.46263358539776134, 0.83419616952574083};
        // setConfigurationSrvClient(final_configuration);

        // std::vector<double> torque = {
        //     -0.00975138, -0.109288, -0.0506026, -0.0190779, 
        //     8.67362e-19, -0.107871, -0.0565052, -0.0254919, 
        //     0.00425203, -0.0427226, -0.0485737, -0.0252509, 
        //     0.00397767, -0.0139315, 0.0208869, 0.00295757
        // };
        // setTorqueSrvClient(torque);

        // std::vector<double> gravity_vector = {0, 0, 0};
        // setGravityVectorClient(gravity_vector);

        // coupledClosingClient(5);
        // std::vector<double> fingertip_positions = {
        //     0.0692667, -0.0261614, 0.0921007, 0.0732823, -0.00199746, 0.111011, 
        //     0.0545853, 0.032039, 0.0967878, 0.0754284, -0.100979, -0.0230322
        // };
        // std::vector<double> fingertip_euler_angles = {
        //     0.330828, 1.30504, 0.0849245, 3.11434, -0.933611, -3.14159, 
        //     2.81657, -1.25037, 3.05522, 3.02704, -0.83002, -1.19531
        // };


        // setPoseIKClient(fingertip_positions, fingertip_euler_angles);

    }

    // Get Controller server client
    void getControllerSrvClient(){
        client_get_controller = this->create_client<allegro_hand_interfaces::srv::GetController>(this->prefix_ + "/getController");

        auto request = std::make_shared<allegro_hand_interfaces::srv::GetController::Request>();
        
        while (!client_get_controller->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_get_controller->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Get Gravity Vector client
    void getGravityVectorSrvClient(){
        client_get_gravity_vector = this->create_client<allegro_hand_interfaces::srv::GetGravityVector>(this->prefix_ + "/getGravityVector");

        auto request = std::make_shared<allegro_hand_interfaces::srv::GetGravityVector::Request>();
        
        while (!client_get_gravity_vector->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_get_gravity_vector->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
            RCLCPP_INFO(this->get_logger(), "Gravity vector: (%f, %f, %f)", result->gravity_vector[0], result->gravity_vector[1], result->gravity_vector[2]);

        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Get Hand Configuration client
    void getHandConfigurationSrvClient(){
        client_get_configuration = this->create_client<allegro_hand_interfaces::srv::GetHandConfiguration>(this->prefix_ + "/getHandConfiguration");

        auto request = std::make_shared<allegro_hand_interfaces::srv::GetHandConfiguration::Request>();
        
        while (!client_get_configuration->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_get_configuration->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
            std::stringstream ss;
            ss << "Current configuration: ";
            for (auto number : result->current_configuration) {
                ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Get Fingertip Poses FK client
    void getPoseFKSrvClient(){
        client_get_pose_fk = this->create_client<allegro_hand_interfaces::srv::GetPoseFK>(this->prefix_ + "/getPoseFK");

        auto request = std::make_shared<allegro_hand_interfaces::srv::GetPoseFK::Request>();
        
        while (!client_get_pose_fk->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_get_pose_fk->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
            if(result->success){
                std::stringstream ss;
                ss << "Coordinates: ";
                for (auto number : result->translation) {
                    ss << number << " ";
                }
                ss << "Orientations: ";
                for (auto number : result->euler_angles) {
                    ss << number << " ";
                }
                RCLCPP_INFO(this->get_logger(), ss.str().c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Get Torque client
    void getTorqueSrvClient(){
        client_get_torque = this->create_client<allegro_hand_interfaces::srv::GetTorque>(this->prefix_ + "/getTorque");

        auto request = std::make_shared<allegro_hand_interfaces::srv::GetTorque::Request>();
        
        while (!client_get_torque->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_get_torque->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
            std::stringstream ss;
            ss << "Torque: ";
            for (auto number : result->torque) {
                ss << number << " ";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Get JointLimits client
    void getJointLimitsSrvClient(){
        client_joint_limits = this->create_client<allegro_hand_interfaces::srv::JointLimits>(this->prefix_ + "/getLimits");

        auto request = std::make_shared<allegro_hand_interfaces::srv::JointLimits::Request>();
        
        while (!client_joint_limits->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_joint_limits->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            std::stringstream ss;
            ss << "Joint limits: \n";
            for(size_t i = 0; i < result->min.size(); ++i){
            ss << "Joint " << i << " min: " << result->min[i] << ", max: " << result->max[i] << "\n";
            }
            RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Set Controller server client
    void setControllerSrvClient(std::string name_controller){
        client_set_controller = this->create_client<allegro_hand_interfaces::srv::SetController>(this->prefix_ + "/setController");

        auto request = std::make_shared<allegro_hand_interfaces::srv::SetController::Request>();
        request->tipo = name_controller;
        while (!client_set_controller->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_set_controller->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Set Configuration server client
    void setConfigurationSrvClient(std::vector<double> desired_configuration){
        client_set_des_pos = this->create_client<allegro_hand_interfaces::srv::SetDesPos>(this->prefix_ + "/setDesiredConfiguration");

        auto request = std::make_shared<allegro_hand_interfaces::srv::SetDesPos::Request>();
        request->pos = desired_configuration;
        while (!client_set_des_pos->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_set_des_pos->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }


    // Set Torque server client
    void setTorqueSrvClient(std::vector<double> torque){
        client_set_des_torque = this->create_client<allegro_hand_interfaces::srv::SetDesTorque>(this->prefix_ + "/setTorque");

        auto request = std::make_shared<allegro_hand_interfaces::srv::SetDesTorque::Request>();
        request->torque = torque;
        while (!client_set_des_torque->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_set_des_torque->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Set Torque server client
    void setGravityVectorClient(std::vector<double> gravity_vector){
        client_set_gravity_vector = this->create_client<allegro_hand_interfaces::srv::SetGravityVector>(this->prefix_ + "/setGravityVector");

        auto request = std::make_shared<allegro_hand_interfaces::srv::SetGravityVector::Request>();
        request->gravity_vector = gravity_vector;
        while (!client_set_gravity_vector->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_set_gravity_vector->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
    
    // Coupled closing client
    void coupledClosingClient(double coupled_value){
        client_coupled_closing = this->create_client<allegro_hand_interfaces::srv::CoupledClosing>(this->prefix_ + "/coupledClosing");

        auto request = std::make_shared<allegro_hand_interfaces::srv::CoupledClosing::Request>();
        request->coupled_value = coupled_value;
        while (!client_coupled_closing->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_coupled_closing->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Set fingertip Pose using IK client
    void setPoseIKClient(std::vector<double> desired_position, std::vector<double> desired_orientation){
        client_set_pose_ik = this->create_client<allegro_hand_interfaces::srv::SetPoseIK>(this->prefix_ + "/setPoseIK");

        auto request = std::make_shared<allegro_hand_interfaces::srv::SetPoseIK::Request>();
        request->desired_position = desired_position;
        request->desired_orientation = desired_orientation;
        while (!client_set_pose_ik->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_set_pose_ik->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Pause a finger client (only if it is moving along a trajectory using an action)
    void pauseFingerClient(unsigned int finger_id){
        client_pause_finger = this->create_client<allegro_hand_interfaces::srv::PauseFinger>(this->prefix_ + "/pauseFinger");

        auto request = std::make_shared<allegro_hand_interfaces::srv::PauseFinger::Request>();
        request->finger_id = finger_id;
        while (!client_pause_finger->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_pause_finger->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // Continue finger client (only if it is moving along a trajectory using an action)
    void continueFingerClient(unsigned int finger_id){
        client_continue_finger = this->create_client<allegro_hand_interfaces::srv::ContinueFinger>(this->prefix_ + "/continueFinger");

        auto request = std::make_shared<allegro_hand_interfaces::srv::ContinueFinger::Request>();
        request->finger_id = finger_id;
        while (!client_continue_finger->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_continue_finger->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }
    

   // stop finger client (only if it is moving along a trajectory using an action)
    void stopFingerClient(unsigned int finger_id){
        client_stop_finger = this->create_client<allegro_hand_interfaces::srv::StopFinger>(this->prefix_ + "/stopFinger");

        auto request = std::make_shared<allegro_hand_interfaces::srv::StopFinger::Request>();
        request->finger_id = finger_id;
        while (!client_stop_finger->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_stop_finger->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    // stop finger client (only if it is moving along a trajectory using an action)
    void stopHandClient(){
        client_stop_hand = this->create_client<allegro_hand_interfaces::srv::StopHand>(this->prefix_ + "/stopHand");

        auto request = std::make_shared<allegro_hand_interfaces::srv::StopHand::Request>();
        while (!client_stop_hand->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        auto result_future = client_stop_hand->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) == 
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s", result->success ? "true" : "false");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

private:
    std::string prefix_;

    rclcpp::Client<allegro_hand_interfaces::srv::GetController>::SharedPtr client_get_controller;
    rclcpp::Client<allegro_hand_interfaces::srv::GetGravityVector>::SharedPtr client_get_gravity_vector;
    rclcpp::Client<allegro_hand_interfaces::srv::GetHandConfiguration>::SharedPtr client_get_configuration;
    rclcpp::Client<allegro_hand_interfaces::srv::GetPoseFK>::SharedPtr client_get_pose_fk;
    rclcpp::Client<allegro_hand_interfaces::srv::GetTorque>::SharedPtr client_get_torque;
    rclcpp::Client<allegro_hand_interfaces::srv::JointLimits>::SharedPtr client_joint_limits;
    rclcpp::Client<allegro_hand_interfaces::srv::SetController>::SharedPtr client_set_controller;
    rclcpp::Client<allegro_hand_interfaces::srv::SetDesPos>::SharedPtr client_set_des_pos;
    rclcpp::Client<allegro_hand_interfaces::srv::SetDesTorque>::SharedPtr client_set_des_torque;
    rclcpp::Client<allegro_hand_interfaces::srv::SetGravityVector>::SharedPtr client_set_gravity_vector;
    rclcpp::Client<allegro_hand_interfaces::srv::CoupledClosing>::SharedPtr client_coupled_closing;
    rclcpp::Client<allegro_hand_interfaces::srv::SetPoseIK>::SharedPtr client_set_pose_ik;
    rclcpp::Client<allegro_hand_interfaces::srv::PauseFinger>::SharedPtr client_pause_finger;
    rclcpp::Client<allegro_hand_interfaces::srv::ContinueFinger>::SharedPtr client_continue_finger;
    rclcpp::Client<allegro_hand_interfaces::srv::StopFinger>::SharedPtr client_stop_finger;
    rclcpp::Client<allegro_hand_interfaces::srv::StopHand>::SharedPtr client_stop_hand;
};  

// This main only works is the allegro_hand_ros node is running.
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string prefix = "";
    
    if (argc == 2) {
        prefix = (argv[1]);
    }

    bool stop = false;
    int option;
    auto server_client = std::make_shared<ServiceClient>(prefix, rclcpp::NodeOptions());
    std::string chain;
    std::cout << "Hand initialized" << std::endl;
    std::cout << "Prefix: " << prefix << std::endl;
    while(!stop){
        std::cout << std::endl;
        std::cout << "   UPC-ALLEGRO HAND APPLICATION" << std::endl;
        std::cout << std::endl;
        std::cout << "Please select one option to continue: " << std::endl;
        std::cout << std::endl;
        std::cout << "<1> Get controller" << std::endl;
        std::cout << "<2> Get gravity vector" << std::endl;
        std::cout << "<3> Get Fingertips position and orientation (FK)" << std::endl;
        std::cout << "<4> Get current torque" << std::endl;
        std::cout << "<5> Get current configuration" << std::endl;
        std::cout << "<6> Get joint limits" << std::endl;
        std::cout << "<7> Set controller" << std::endl;
        std::cout << "<8> Move to a desired configuration" << std::endl;
        std::cout << "<9> Set desired torque" << std::endl;
        std::cout << "<G> Set gravity vector" << std::endl;
        std::cout << "<C> Coupled closing" << std::endl;
        std::cout << "<I> Move fingertips to a pose. Compute IK" << std::endl;
        std::cout << "<W> Pause Finger Movement" << std::endl;
        std::cout << "<E> Continue Finger Movement" << std::endl;
        std::cout << "<Q> Stop Finger Movement" << std::endl;
        std::cout << "<S> Stop Hand" << std::endl;

        std::cout << "<0> EXIT" << std::endl;
        std::cout<<std::endl;

        system("stty raw");
        option=getchar();
        system("stty cooked");
        std::cout << std::endl;

        switch (option){
        case '1':
        {
            // Get controller
            server_client->getControllerSrvClient();  
            break;
        }
        case '2':
        {
            //Get gravity vector
            server_client->getGravityVectorSrvClient();
            break;
        }
        case '3':
        {
            // Get fingertip positions and orientations
            server_client->getPoseFKSrvClient();
            break;
        }
        case '4':
        {
            // Get torque
            server_client->getTorqueSrvClient();
            break;
        }
        case '5':
        {
            // Get current hand configuration
            server_client->getHandConfigurationSrvClient();
            break;
        }
        case '6':
        {
            // Get joint limits
            server_client->getJointLimitsSrvClient();
            break;
        }
        case '7':
        {
            std::cout << "Enter controller name: JointPositionPID, GravityCompensator, Torque" << std::endl;
            getline(std::cin,chain);
            server_client->setControllerSrvClient(chain);
            break;
        }
        case '8':
        {
            std::vector<double> final_configuration = {-0.35780429446246664, 0.036747947880789181, 0.82771645890907997, 0.4559763484628358, 0.025652552989246551, 0.10181134352479515, 0.66847535142566017, 0.16509947598615426, 0.33596855731591074, -0.17433084453591777, 0.90485164419508435, 0.53728340222806015, 0.3259383203339562, 0.76771256333561744, 0.46263358539776134, 0.83419616952574083};
            server_client->setConfigurationSrvClient(final_configuration);
            break;
        }
        case '9':
        {
            std::vector<double> torque = {
                -0.00975138, -0.109288, -0.0506026, -0.0190779, 
                8.67362e-19, -0.107871, -0.0565052, -0.0254919, 
                0.00425203, -0.0427226, -0.0485737, -0.0252509, 
                0.00397767, -0.0139315, 0.0208869, 0.00295757
            };
            server_client->setTorqueSrvClient(torque);
            break;
        }
        case 'g':
        case 'G':
        {
            std::vector<double> gravity_vector = {0, 0, 0};
            server_client->setGravityVectorClient(gravity_vector);
            break;
        }
        case 'c':
        case 'C':
        {
            server_client->coupledClosingClient(5);
            break;
        }
        case 'i':
        case 'I':
        {
            std::vector<double> fingertip_positions = {
                0.0692667, -0.0261614, 0.0921007, 0.0732823, -0.00199746, 0.111011, 
                0.0545853, 0.032039, 0.0967878, 0.0754284, -0.100979, -0.0230322
            };
            std::vector<double> fingertip_euler_angles = {
                0.330828, 1.30504, 0.0849245, 3.11434, -0.933611, -3.14159, 
                2.81657, -1.25037, 3.05522, 3.02704, -0.83002, -1.19531
            };
            server_client->setPoseIKClient(fingertip_positions, fingertip_euler_angles);
            break;
        }
        case 'w':
        case 'W':
        {
            server_client->pauseFingerClient(0);
            break;
        }
        case 'e':
        case 'E':
        {
            server_client->continueFingerClient(0);
            break;
        }
        case 'q':
        case 'Q':
        {
            server_client->stopFingerClient(0);
            break;
        }
        case 's':
        case 'S':
        {
            server_client->stopHandClient();
            break;
        }
        case '0':
        {
            stop = true;
            break;
        }
        default:
            break;
        }
    }
    std::cout << "PROGRAM FINISHED!!!!" << std::endl;
    rclcpp::shutdown();   
    return 0;
}