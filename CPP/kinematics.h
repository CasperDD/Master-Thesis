#include <cmath>
#include <vector>
#include <gsl/gsl_linalg.h>

class Kinematics
{
private:
    double l = 15;                      // distance between wheels
    double wheel_circumference = 18.85; // circumference of wheel

    double setTicsToRotate(int pwm)
    {
        return -0.74 * pwm + 600.15;
    }

    double setCenterOfWheelBase(double pwm)
    {
        return (l / wheel_circumference) * setTicsToRotate(pwm);
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
    std::vector<int> left_encoder_tics;
    std::vector<int> right_encoder_tics;
    int left_tics = 0;
    int right_tics = 0;
    std::vector<double> time_point;
    gsl_vector *X_Y_Theta = gsl_vector_alloc(3);

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

    std::vector<double> directionVector()
    {
        std::vector<double> direction_vector;
        double tic_length = sqrt(pow(gsl_vector_get(X_Y_Theta, 0), 2) + pow(gsl_vector_get(X_Y_Theta, 1), 2));

        double angle_to_origin = asin(gsl_vector_get(X_Y_Theta, 1) / tic_length);

        double turn_angle = M_PI + (gsl_vector_get(X_Y_Theta, 2) - angle_to_origin);

        if (turn_angle > M_PI)
        {
            turn_angle = turn_angle - M_PI * 2;
        }

        direction_vector.push_back(tic_length);
        direction_vector.push_back(turn_angle);

        // std::cout << "direction vector angle: " << tic_l ength << std::endl;

        return direction_vector;
    }
};