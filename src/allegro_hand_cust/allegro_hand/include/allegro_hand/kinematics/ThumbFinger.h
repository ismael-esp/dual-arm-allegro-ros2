#ifndef THUMBFINGER_H_
#define THUMBFINGER_H_

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "AllegroFinger.h"

// Rotation part of the transformation from the world coordinates to the base of each finger
const std::vector<double> eul_ang_w2thumbBase = { 1.5760,   -0.0868,   -1.5794};        //Eul angles transformation for the index finger. Transformation from base_link to link_0_0 according to tf_echo

// Translation part of the transformation from the world coordinates to the base of each finger
const std::vector<double> pos_w2b_thumbBase = {-0.0182, -0.0170, -0.0729};                 //Eul angles transformation for the middle finger. Transformation from base_link to link_8_0 according to xacro file

class ThumbFinger: public AllegroFinger
{

  public:
    //Constructor of the thumb Allegro finger
    ThumbFinger();
    ThumbFinger(std::vector<double> link_lengths, std::vector<double> q_min_limits_, std::vector<double> q_max_limits_);
        
    std::vector<std::vector<double>> IkineThumb(std::vector<double> goal_eul_ang, std::vector<double> goal_pos);
    bool IkineThumb(std::vector<double> goal_eul_ang, std::vector<double> goal_pos, std::vector<std::vector<double>> &q_values_ikine);
    //Ikine given a normal and a position
    // bool IkineNormal(std::vector<double> goal_normal, std::vector<double> goal_pos);
        
    std::vector<std::vector<double>> considerValidJointValues(std::vector<std::vector<double>> partial_q_values, bool four_elements);
    bool checkPointInsideWs(double px, double py, double pz);
    std::vector<double> analyze_results_quality(std::vector<std::vector<double>> final_joint_values, Eigen::Affine3d T_desired, bool *correct_sol_found);


  private:
    int priv_thumb_var;
    
};


#endif // THUMBFINGER_H_
