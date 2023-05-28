#include <cmath>
#include <vector>
#include <gsl/gsl_linalg.h>

class Kinematics
{
private:
    double l = 15;                      // distance between wheels
    double r = 3;                       // radius of wheel
    double wheel_circumference = 18.85; // circumference of wheel

    // Tics for wheel to rotate a full rotation
    // This is found by observing the rotation of the wheel with different PWM
    double setTicsToRotate(int pwm)
    {
        return -0.74 * pwm + 600.15;
    }

    // Set the center of wheel base, used for the kinematic model.
    double setCenterOfWheelBase(double pwm)
    {
        return ((l/2) / wheel_circumference) * setTicsToRotate(pwm);
    }

    void setRotationMatrix(gsl_matrix &rotationmatrix, int i, double pwm)
    {
        double V_l = left_encoder_tics[i] / time_point[i];
        double V_r = right_encoder_tics[i] / time_point[i];
        double center_of_wheel_base = setCenterOfWheelBase(pwm);
        double omega = (V_r - V_l) / center_of_wheel_base;
        double dt = time_point[i];

        gsl_matrix_set(&rotationmatrix, 0, 0, cos(omega * dt));
        gsl_matrix_set(&rotationmatrix, 0, 1, -sin(omega * dt));
        gsl_matrix_set(&rotationmatrix, 0, 2, 0);
        gsl_matrix_set(&rotationmatrix, 1, 0, sin(omega * dt));
        gsl_matrix_set(&rotationmatrix, 1, 1, cos(omega * dt));
        gsl_matrix_set(&rotationmatrix, 1, 2, 0);
        gsl_matrix_set(&rotationmatrix, 2, 0, 0);
        gsl_matrix_set(&rotationmatrix, 2, 1, 0);
        gsl_matrix_set(&rotationmatrix, 2, 2, 1);
    }

    void setTranslationVector(gsl_vector &translation, int i, double pwm)
    {
        double V_l = left_encoder_tics[i] / time_point[i];
        double V_r = right_encoder_tics[i] / time_point[i];
        double center_of_wheel_base = setCenterOfWheelBase(pwm);
        double R = (center_of_wheel_base / 2) * ((V_l + V_r) / (V_r - V_l));

        if (V_r - V_l == 0)
        {
            R = 0;
        }

        double ICC_x = gsl_vector_get(X_Y_Theta, 0) - R * sin(gsl_vector_get(X_Y_Theta, 2));
        double ICC_y = gsl_vector_get(X_Y_Theta, 1) + R * cos(gsl_vector_get(X_Y_Theta, 2));

        gsl_vector_set(&translation, 0, gsl_vector_get(X_Y_Theta, 0) - ICC_x);
        gsl_vector_set(&translation, 1, gsl_vector_get(X_Y_Theta, 1) - ICC_y);
        gsl_vector_set(&translation, 2, gsl_vector_get(X_Y_Theta, 2));
    }

    void setICCVector(gsl_vector &ICC, int i, double pwm)
    {
        double V_l = left_encoder_tics[i] / time_point[i];
        double V_r = right_encoder_tics[i] / time_point[i];
        double center_of_wheel_base = setCenterOfWheelBase(pwm);
        double omega = (V_r - V_l) / center_of_wheel_base;
        double dt = time_point[i];
        double R = (center_of_wheel_base / 2) * ((V_l + V_r) / (V_r - V_l));

        if (V_r - V_l == 0)
        {
            R = 0;
        }

        double ICC_x = gsl_vector_get(X_Y_Theta, 0) - R * sin(gsl_vector_get(X_Y_Theta, 2));
        double ICC_y = gsl_vector_get(X_Y_Theta, 1) + R * cos(gsl_vector_get(X_Y_Theta, 2));
        // std::cout << "omega " << omega << " dt " << dt << " * " << omega * dt << std::endl;
        gsl_vector_set(&ICC, 0, ICC_x);
        gsl_vector_set(&ICC, 1, ICC_y);
        gsl_vector_set(&ICC, 2, omega * dt);
    }

public:
    std::vector<int> left_encoder_tics; //left encoder values used to calculate the kinematics and save the "route" the robot have went
    std::vector<int> right_encoder_tics; //right encoder values used to calculate the kinematics and save the "route" the robot have went
    int left_tics = 0; //Used to check how much the robot have went on the left wheel
    int right_tics = 0; //Used to check how much the robot have went on the right wheel
    int pirouette_left = 0; //Used to check how much the robot have went on the left wheel, during the pirouette
    int pirouette_right = 0; //Used to check how much the robot have went on the right wheel, during the pirouette
    std::vector<double> time_point; //The time used to calculate the velocity
    gsl_vector *X_Y_Theta = gsl_vector_alloc(3); //Vector containing the X, Y and theta of the robot heading

    //Calculate the heading of the robot, using the kinematic
    void positionDirection(double pwm)
    {
        gsl_matrix *rotation_matrix = gsl_matrix_alloc(3, 3);
        gsl_vector *translation = gsl_vector_alloc(3);
        gsl_vector *ICC = gsl_vector_alloc(3);

        for (int i = 0; i < left_encoder_tics.size(); i++)
        {
            setRotationMatrix(*rotation_matrix, i, pwm);

            setTranslationVector(*translation, i, pwm);

            setICCVector(*ICC, i, pwm);

            gsl_blas_dgemv(CblasNoTrans, 1.0, rotation_matrix, translation, 0, X_Y_Theta);
            gsl_blas_daxpy(1.0, ICC, X_Y_Theta);
        }
        gsl_matrix_free(rotation_matrix);
        gsl_vector_free(translation);
        gsl_vector_free(ICC);
    }

    //Make a direction vector that points towards home
    //Find the amount the robot needs to turn to hit that direction vector
    std::vector<double> directionVector()
    {
        // std::cout << "x_y_theta the theta value: " << gsl_vector_get(X_Y_Theta, 2) << std::endl;
        std::vector<double> direction_vector;
        double tic_length = sqrt(pow(gsl_vector_get(X_Y_Theta, 0), 2) + pow(gsl_vector_get(X_Y_Theta, 1), 2));

        double x = gsl_vector_get(X_Y_Theta, 0);
        double y = gsl_vector_get(X_Y_Theta, 1);

        double angle_to_origin = asin(gsl_vector_get(X_Y_Theta, 1) / tic_length);

        double turn_angle = M_PI + (gsl_vector_get(X_Y_Theta, 2) - angle_to_origin);
        
        std::cout << "before turn angle: " << turn_angle << std::endl;

        if (turn_angle > M_PI)
        {
            turn_angle = turn_angle - 2 * M_PI;
        }

        direction_vector.push_back(tic_length);
        direction_vector.push_back(turn_angle);

        std::cout << "after turn angle: " << turn_angle << std::endl;
        // std::cout << "Angle to origin: " << angle_to_origin << std::endl;

        return direction_vector;
    }

    //Reset the position
    void clear_XYTheta()
    {
        gsl_vector_set_zero(X_Y_Theta);
    }
};