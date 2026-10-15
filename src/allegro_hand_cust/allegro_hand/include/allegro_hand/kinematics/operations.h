#include "AllegroFinger.h"

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

void printVector(std::vector<double> vect);
Eigen::Matrix<double, 3, 1> obtainEulAngles(Eigen::Affine3d mat);
Eigen::Quaterniond obtainQuaternion(Eigen::Affine3d mat);
Eigen::Matrix<double, 3, 1> obtainTranslation(Eigen::Affine3d mat);

// Function for solving the system of equations
double WrapPosNegPI(double fAng);
double WrapTwoPI(double fAng);
double eval_f1(double q1, double q2, double q3, double px);
double eval_f2(double q1, double q2, double q3, double py);
double eval_f3(double q2, double q3, double pz);
Eigen::MatrixXd eval_jacabian(double q1, double q2, double q3);
std::vector <Eigen::Vector3d> systemEquationsSolver(double px, double py, double pz);
