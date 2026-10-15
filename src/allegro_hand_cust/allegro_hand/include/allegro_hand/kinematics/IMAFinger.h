#ifndef IMAFinger_H_
#define IMAFinger_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "AllegroFinger.h"

class IMAFinger: public AllegroFinger
{

  public:
    //Constructor of the desired Allegro finger
    IMAFinger(int finger_selection_);
    IMAFinger(unsigned int finger_selection_, std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_);
    int getValue();
  
  private:
    int priv_IMA_var;


};


#endif // IMAFinger_H_
