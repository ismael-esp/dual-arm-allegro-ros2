#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <random>
#include <vector>

//https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi

static const double     _PI= 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348;
static const double _TWO_PI= 6.2831853071795864769252867665590057683943387987502116419498891846156328125724179972560696;

// Floating-point modulo
// The result (the remainder) has same sign as the divisor.
// Similar to matlab's mod(); Not similar to fmod() -   Mod(-3,4)= 1   fmod(-3,4)= -3
template<typename T>
T Mod(T x, T y)
{
    static_assert(!std::numeric_limits<T>::is_exact , "Mod: floating-point type expected");

    if (0. == y)
        return x;

    double m= x - y * floor(x/y);

    // handle boundary cases resulted from floating-point cut off:

    if (y > 0)              // modulo range: [0..y)
    {
        if (m>=y)           // Mod(-1e-16             , 360.    ): m= 360.
            return 0;

        if (m<0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(106.81415022205296 , _TWO_PI ): m= -1.421e-14 
        }
    }
    else                    // modulo range: (y..0]
    {
        if (m<=y)           // Mod(1e-16              , -360.   ): m= -360.
            return 0;

        if (m>0 )
        {
            if (y+m == y)
                return 0  ; // just in case...
            else
                return y+m; // Mod(-106.81415022205296, -_TWO_PI): m= 1.421e-14 
        }
    }

    return m;
}

// wrap [rad] angle to [-PI..PI)
inline double WrapPosNegPI(double fAng)
{
    return Mod(fAng + _PI, _TWO_PI) - _PI;
}

// wrap [rad] angle to [0..TWO_PI)
inline double WrapTwoPI(double fAng)
{
    return Mod(fAng, _TWO_PI);
}

double eval_f1(double q1, double q2, double q3, double px)
{
    double result;
    result = 0.0050*cos(q1) + 0.0576*sin(q1) + 0.0514*cos(q3)*sin(q1) + 0.0514*cos(q1)*sin(q2)*sin(q3) - px;
    return result;
}

double eval_f2(double q1, double q2, double q3, double py)
{
    double result;
    result = 0.0050*sin(q1) - 0.0576*cos(q1) - 0.0514*cos(q1)*cos(q3) + 0.0514*sin(q1)*sin(q2)*sin(q3) -py;
    return result;
}

double eval_f3(double q2, double q3, double pz)
{
    double result;
    result = 0.0514*cos(q2)*sin(q3) - pz;
    return result;
}


Eigen::MatrixXd eval_jacabian(double q1, double q2, double q3)
{
    /*J11 J12 J13
    J21 J22 J23
    J31 J32 J33
    */
    
    Eigen::MatrixXd J = Eigen::Matrix3d(3,3);
    J(0,0) = -0.0050*sin(q1)+0.0576*cos(q1)+0.0514*cos(q3)*cos(q1)-0.0514*sin(q1)*sin(q2)*sin(q3);
    J(0,1) = 0.0514*cos(q1)*cos(q2)*sin(q3);
    J(0,2) = -0.0514*sin(q3)*sin(q1)+0.0514*cos(q1)*sin(q2)*cos(q3);
    J(1,0) = 0.0050*cos(q1)+0.0576*sin(q1)+0.0514*cos(q3)*sin(q1)+0.0514*cos(q1)*sin(q2)*sin(q3);
    J(1,1) = 0.0514*sin(q1)*cos(q2)*sin(q3);
    J(1,2) = 0.0514*sin(q3)*cos(q1)+0.0514*sin(q1)*sin(q2)*cos(q3);
    J(2,0) = 0;
    J(2,1) = -0.0514*sin(q2)*sin(q3);
    J(2,2) = 0.0514*cos(q2)*cos(q3);
    return J;
}

int main(int argc, char** argv) {

    if (argc < 4){
        std::cout << "Syntax: solver px py pz " << std::endl;
        std::cout << "Please provide the desired values of px, py and pz" << std::endl;
        return 1;
    }
    Eigen::MatrixXd jacobian = Eigen::Matrix3d(3,3);
    Eigen::Vector3d X_i, X_i1, F, tmp;
    std::vector <double> E;
    E.resize(3);
    std::vector <Eigen::Vector3d> solutions;
    
    
    // random numbers
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    // limits
    std::vector<double> max_limits = {0.47, 1.61, 1.71};
    std::vector<double> min_limits = {-0.47, -0.2, -0.17};
    
    std::uniform_real_distribution<> dist_q1(min_limits[0],max_limits[0]); 
    std::uniform_real_distribution<> dist_q2(min_limits[1],max_limits[1]); 
    std::uniform_real_distribution<> dist_q3(min_limits[2],max_limits[2]); 
        
    X_i(0) = dist_q1(gen);
    X_i(1) = dist_q2(gen);
    X_i(2) = dist_q3(gen);
    
    // initial values    
    double px, py, pz;
    px = std::atof(argv[1]);
    py = std::atof(argv[2]);
    pz = std::atof(argv[3]);
        
    //epsilon
    double epsilon = 1e-6;
    double error = 1e1; //initial value
    
    unsigned int iter = 0;
    unsigned int attempts = 0;
    unsigned int max_attempts = 30;
    bool end = false;
    
    while(!end)
    {
        X_i(0) = dist_q1(gen);
        X_i(1) = dist_q2(gen);
        X_i(2) = dist_q3(gen);
        error = 1e1;

        while (error > epsilon && iter < 1000)
        {
            jacobian = eval_jacabian(X_i(0), X_i(1), X_i(2));
            F(0) = eval_f1( X_i(0), X_i(1), X_i(2), px);
            F(1) = eval_f2( X_i(0), X_i(1), X_i(2), py);
            F(2) = eval_f3( X_i(1), X_i(2), pz);
       
            X_i1 = X_i - jacobian.inverse() * F; 
            E[0] = fabs(fabs(X_i1(0)) - fabs(X_i(0)));
            E[1] = fabs(fabs(X_i1(1)) - fabs(X_i(1)));
            E[2] = fabs(fabs(X_i1(2)) - fabs(X_i(2)));
            auto max_iter = std::max_element(E.begin(), E.end());
            error = *max_iter;
        
            iter++;
            X_i = X_i1;        
    
        }
        if(iter < 1000) // solution
        {
            // normalize
            X_i(0) = WrapPosNegPI(X_i(0));
            X_i(1) = WrapPosNegPI(X_i(1));
            X_i(2) = WrapPosNegPI(X_i(2));
            // first case
            if(attempts == 0)
            {
               solutions.push_back(X_i);
               attempts = 1;
               /*std::cout << "q1 = " << X_i(0) << std::endl;
               std::cout << "q2 = " << X_i(1) << std::endl;
               std::cout << "q3 = " << X_i(2) << std::endl; */
            }
            else //we have more solutions
            {
                attempts++;
                bool found = false;
                unsigned int i = 0;
                while(!found && i<solutions.size())
                {
                    tmp = solutions[i] - X_i;
                    if(fabs(tmp.norm()) < epsilon)
                    {
                       found = true;
                    }
                    i++;
                }
                if(!found)
                {
                    solutions.push_back(X_i);
                    /*std::cout << "q1 = " << X_i(0) << std::endl;
                    std::cout << "q2 = " << X_i(1) << std::endl;
                    std::cout << "q3 = " << X_i(2) << std::endl; */
                }
            }
        }
        if (attempts > max_attempts)
                end = true;
        attempts++;
    }
    std::cout << "Found these solutions: " << std::endl;
    
    for(unsigned int i = 0; i < solutions.size();i++)
    {
        std::cout << "q1 = " << solutions[i](0) << std::endl;
        std::cout << "q2 = " << solutions[i](1) << std::endl;
        std::cout << "q3 = " << solutions[i](2) << std::endl;
        std::cout << "Evaluating the solution: f1=" <<  eval_f1( solutions[i](0), solutions[i](1), solutions[i](2), px) << std::endl; 
        std::cout << "                         f2=" <<  eval_f2( solutions[i](0), solutions[i](1), solutions[i](2), py) << std::endl; 
        std::cout << "                         f3=" <<  eval_f3( solutions[i](1), solutions[i](2), pz) << std::endl; 
        std::cout << "------------------------------" << std::endl;
    }
    return 0;
}

