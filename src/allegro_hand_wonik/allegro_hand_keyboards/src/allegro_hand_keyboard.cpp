#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

#include "virtualkey_codes.h"

using namespace std;

#define DOF_JOINTS 16


class AHKeyboard : public rclcpp::Node
{
public:
  AHKeyboard();
  void keyLoop();
  void printUsage();

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
};

AHKeyboard::AHKeyboard() : Node("allegro_hand_keyboard")
{
  cmd_pub_ = this->create_publisher<std_msgs::msg::String>("allegroHand/lib_cmd", 10);
}


int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AHKeyboard>();

  signal(SIGINT, quit);

  node->keyLoop();
  rclcpp::shutdown();

  return 0;
}

void AHKeyboard::printUsage() {
  std::cout << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  std::cout << "  Use the keyboard to send Allegro Hand grasp & motion commands" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;

  std::cout << "\tHome Pose:\t\t\t'H'" << std::endl;
  std::cout << "\tReady Pose:\t\t\t'R'" << std::endl;
  std::cout << "\tPinch (index+thumb):\t\t'P'" << std::endl;
  std::cout << "\tPinch (middle+thumb):\t\t'M'" << std::endl;
  std::cout << "\tGrasp (3 fingers):\t\t'G'" << std::endl;
  std::cout << "\tGrasp (4 fingers):\t\t'F'" << std::endl;
  std::cout << "\tGrasp (envelop):\t\t'E'" << std::endl;
  std::cout << "\tGravity compensation:\t\t'Z'" << std::endl;
  std::cout << "\tMotors Off (free motion):\t'O'" << std::endl;
  std::cout << "\tSave Current Pose:\t\t'S'" << std::endl;
  std::cout << "\tPD Control (last saved):\t'Space'" << std::endl;
  std::cout << "\tHelp (this message):\t\t'/ or ?'" << std::endl;
  std::cout << " -----------------------------------------------------------------------------" << std::endl;
  //std::cout << "  Note: Unless elsewhere implemented, these keyboard commands only work with " << std::endl;
  //std::cout << "  the 'allegro_hand_core_grasp' and 'allegro_hand_core_grasp_slp' packages." << std::endl;
  std::cout << "  Subscriber code for reading these messages is included in '~core_template'." << std::endl;
  std::cout << " -----------------------------------------------------------------------------\n" << std::endl;

}

void AHKeyboard::keyLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  sleep(2);
  printUsage();

  for(;;)
  {
    std_msgs::msg::String msg;
    std::stringstream ss;

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    RCLCPP_DEBUG(this->get_logger(), "value: 0x%02X", c);
    switch(c)
    {
      case VK_SPACE:
        RCLCPP_DEBUG(this->get_logger(), "space bar: PD Control");
        ss << "pdControl";
        dirty = true;
        break;
      case KEYCODE_h:
        RCLCPP_DEBUG(this->get_logger(), "h_key: Home");
        ss << "home";
        dirty = true;
        break;
      case KEYCODE_r:
        RCLCPP_DEBUG(this->get_logger(), "r_key: Ready");
        ss << "ready";
        dirty = true;
        break;
      case KEYCODE_g:
        RCLCPP_DEBUG(this->get_logger(), "g_key: Grasp (3 finger)");
        ss << "grasp_3";
        dirty = true;
        break;
      case KEYCODE_f:
        RCLCPP_DEBUG(this->get_logger(), "f_key: Grasp (4 finger)");
        ss << "grasp_4";
        dirty = true;
        break;
      case KEYCODE_p:
        RCLCPP_DEBUG(this->get_logger(), "p_key: Pinch (index)");
        ss << "pinch_it";
        dirty = true;
        break;
      case KEYCODE_m:
        RCLCPP_DEBUG(this->get_logger(), "m_key: Pinch (middle)");
        ss << "pinch_mt";
        dirty = true;
        break;
      case KEYCODE_e:
        RCLCPP_DEBUG(this->get_logger(), "e_key: Envelop");
        ss << "envelop";
        dirty = true;
        break;
      case KEYCODE_z:
        RCLCPP_DEBUG(this->get_logger(), "z_key: Gravcomp");
        ss << "gravcomp";
        dirty = true;
        break;
      case KEYCODE_o:
        RCLCPP_DEBUG(this->get_logger(), "o_key: Servos Off");
        ss << "off";
        dirty = true;
        break;
      case KEYCODE_s:
        RCLCPP_DEBUG(this->get_logger(), "s_key: Save joint pos. for PD control");
        ss << "save";
        dirty = true;
        break;
      case KEYCODE_slash:
      case KEYCORD_question:
        printUsage();
        break;
    }

    if(dirty ==true)
    {
      msg.data = ss.str();
      RCLCPP_INFO(this->get_logger(), "%s", msg.data.c_str());
      cmd_pub_->publish(msg);
      rclcpp::spin_some(this->get_node_base_interface());
      dirty = false;
    }
  }

  return;
}
