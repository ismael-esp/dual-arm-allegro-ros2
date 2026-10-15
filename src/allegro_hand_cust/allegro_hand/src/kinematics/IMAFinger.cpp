#include <allegro_hand/kinematics/IMAFinger.h>
#include <allegro_hand/kinematics/AllegroFinger.h>
#include <iostream>

IMAFinger::IMAFinger(int finger_selection_):AllegroFinger(finger_selection_)
{
	// super(finger_selection_);
	// std::cout << "Initializing the finger derived class " <<std::endl;

}

        
IMAFinger::IMAFinger(unsigned int finger_selection_, std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_)
:AllegroFinger(finger_selection_, link_lengths, q_min_limits_, q_max_limits_)
{
	// super(finger_selection_);
	// std::cout << "Initializing the finger derived class " <<std::endl;

}

int IMAFinger::getValue()
{
	return priv_IMA_var;
}
