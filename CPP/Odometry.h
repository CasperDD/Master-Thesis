#include <iostream>
#include <cmath>
#include <vector>
#include <gsl/gsl_linalg.h>

class Odometry
{
private:
    double l = 15;                      // distance between wheels
    double r = 3;                       // radius of wheel
    double wheel_circumference = 18.85; // circumference of wheel

    double setTicsToRotate(int pwm)
    {
        return -0.74 * pwm + 600.15;
    }

    double setCenterOfWheelBase(double pwm)
    {
        return ((l/2) / wheel_circumference) * setTicsToRotate(pwm);
    }

public:
    std::vector<int> left_encoder_tics;
    std::vector<int> right_encoder_tics;
    std::vector<double> time_point;
    double time_diff;
    int left_tics = 0;
    int right_tics = 0;
    int left_tics_total = 0;
    int right_tics_total = 0;
    double x;
    double y;
    double theta;

    void positionDirection(int pwm)
    {
        std::vector<double> direction_vector;
        
        double dt = time_diff;
        double left_dist = wheel_circumference * left_tics / setTicsToRotate(pwm);
        double right_dist = wheel_circumference * right_tics / setTicsToRotate(pwm);
        double dist = (left_dist + right_dist) / 2.0;
        double delta_theta = (right_dist - left_dist) / l;
        double v = dist / dt;
        double omega = delta_theta / dt;

    }
};