#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>
#include <future>
#include <chrono>
#include <numeric>

#include "kinematics.h"
#include "camera.h"
#include "myapriltag.h"
#include "control.h"

class LearningWalk
{
private:
    Kinematics kinematic;
    AprilTagDetector april_tag_detector;
    Camera camera;
    Controller control;

    int center = camera.width / 2;
    int w;
    int h;
    int x;
    bool paused;

    std::vector<std::vector<int>> encoder_values;
    std::vector<std::vector<int>> encoder_tics;

    bool push_back_encode = false;
    bool goal_found = false;
    bool in_center = false;
    bool pause_log = false;

    float learn_rate;
    float pred_weight;
    float reflex_weight;
    float reflex_old;
    float ico_out;

    std::thread log_process;

public:
    LearningWalk()
    {
        gsl_vector_set_zero(kinematic.X_Y_Theta);
    }

    ~LearningWalk() {}

    int counter(std::pair<int, int> prev_state, std::pair<int, int> curr_state, int count)
    {
        if (prev_state == std::make_pair(0, 0))
            {
                if (curr_state == std::make_pair(0, 1))
                {
                    ++count;
                }
                else if (curr_state == std::make_pair(1, 0))
                {
                    --count;
                }
            }
            else if (prev_state == std::make_pair(0, 1))
            {
                if (curr_state == std::make_pair(0, 0))
                {
                    --count;
                }
                else if (curr_state == std::make_pair(1, 1))
                {
                    ++count;
                }
            }
            else if (prev_state == std::make_pair(1, 0))
            {
                if (curr_state == std::make_pair(1, 1))
                {
                    --count;
                }
                else if (curr_state == std::make_pair(0, 0))
                {
                    ++count;
                }
            }
            else if (prev_state == std::make_pair(1, 1))
            {
                if (curr_state == std::make_pair(1, 0))
                {
                    ++count;
                }
                else if (curr_state == std::make_pair(0, 1))
                {
                    --count;
                }
            }
        return count;
    }

    std::vector<int> ticCount(int tail, int head)
    {
        int left_count = 0, right_count = 0;
        std::pair<int, int> prev_state_left(encoder_values[tail][0], encoder_values[tail][1]);
        std::pair<int, int> prev_state_right(encoder_values[tail][2], encoder_values[tail][3]);

        for (int i = tail; i < head; ++i)
        {
            std::pair<int, int> curr_state_left(encoder_values[i][0], encoder_values[i][1]);
            std::pair<int, int> curr_state_right(encoder_values[i][2], encoder_values[i][3]);

            left_count = counter(prev_state_left, curr_state_left, left_count);
            right_count = counter(prev_state_right, curr_state_right, right_count);

            prev_state_left = curr_state_left;
            prev_state_right = curr_state_right;
        }
        std::vector<int> temp;
        temp.push_back(left_count);
        temp.push_back(right_count);

        return temp;
    }

    void loggingProcess()
    {
        // std::cout << " In log encoder " << std::endl;
        int head = 1, tail = 1;
        auto timer = std::chrono::high_resolution_clock::now(); // Create timer to the current instance
        while (true)
        {
            // std::cout << " im in the while loop " << std::endl;
            encoder_values.push_back(control.get_encode_values());
            if (push_back_encode == true)
            {
                // std::cout << "push_back_encode: " << push_back_encode << std::endl;
                tail = head;
                head = encoder_values.size();
                std::cout << "head: " << head << std::endl;
                std::chrono::_V2::high_resolution_clock::time_point end = timer;
                timer = std::chrono::high_resolution_clock::now();
                double time_elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(timer - end).count();
                kinematic.time_point.push_back(time_elapsed);

                std::vector<int> temp = ticCount(tail, head);
                std::cout << "left: " << temp.at(0) * -1 << std::endl;
                std::cout << "right: " << temp.at(1) << std::endl;


                // kinematic.left_encoder_tics.push_back(temp.at(0));
                // kinematic.right_encoder_tics.push_back(temp.at(1));

                kinematic.left_tics += (temp.at(0) * -1);
                kinematic.right_tics += temp.at(1);

                std::cout << "sum left: " << kinematic.left_tics << std::endl;
                std::cout << "sum right: " << kinematic.right_tics << std::endl;
                    
                push_back_encode = false;
                // encoder_values.clear();
            }
        }
    }

    double ICO(double predict, double reflex)
    {
        double deri_reflex = reflex - reflex_old;

        pred_weight += learn_rate * predict * deri_reflex;

        double output = pred_weight * predict + reflex_weight * reflex;

        std::cout << "ICO output: " << output << std::endl;

        reflex_old = reflex;

        return output;
    }

    double updateDirVec()
    {
        kinematic.positionDirection(control.speed);
        vector<double> dir_vec = kinematic.directionVector();
        double theta = dir_vec[1];

        cout << "Theta: " << theta << endl;

        return theta;
    }

    // void pirouette(double theta)
    // {
    //     control.turn(theta);

    //     double x_coord = x_coord_yolo();

    //     while (in_center == false)
    //     {
    //         visionTurn(x_coord);
    //     }
    // }

    // void visionTurn(double x)
    // {
    //     if (x == NULL)
    //     {
    //         kinematic.control.setLeftMotor(0, 1);
    //         kinematic.control.setRightMotor(0, 1);
    //         in_center = false;
    //     }
    //     else if (x < center - 10)
    //     {
    //         kinematic.control.setLeftMotor(kinematic.control.speed, 0);
    //         kinematic.control.setRightMotor(kinematic.control.speed, 1);
    //         in_center = false;
    //     }
    //     else if (x > center + 10)
    //     {
    //         kinematic.control.setLeftMotor(kinematic.control.speed, 1);
    //         kinematic.control.setRightMotor(kinematic.control.speed, 0);
    //         in_center = false;
    //     }
    //     else
    //     {
    //         kinematic.control.setLeftMotor(0, 1);
    //         kinematic.control.setRightMotor(0, 1);
    //         in_center = true;
    //     }
    // }

    // void visionTarget(double x)
    // {
    //     if (x < center - 10)
    //     {
    //         kinematic.control.setLeftMotor(kinematic.control.minSpeed, 1);
    //         kinematic.control.setRightMotor(kinematic.control.maxSpeed, 1);
    //         in_center = false;
    //     }
    //     else if (x > center + 10)
    //     {
    //         kinematic.control.setLeftMotor(kinematic.control.maxSpeed, 1);
    //         kinematic.control.setRightMotor(kinematic.control.minSpeed, 1);
    //         in_center = false;
    //     }
    //     else
    //     {
    //         kinematic.control.setLeftMotor(kinematic.control.speed, 1);
    //         kinematic.control.setRightMotor(kinematic.control.speed, 1);
    //     }
    // }

    // void x_coord_object()
    // {
    //     while (vision_thread)
    //     {
    //         auto scene = camera.getImage();

    //         std::vector<std::vector<int>> box_center = april_tag_detector.detect(scene);

    //         if (box_center.size() > 0)
    //         {
    //             x = box_center[0][0];
    //             w = box_center[0][2];
    //             h = box_center[0][3];
    //         }
    //         else
    //         {
    //             x = NULL;
    //             w = NULL;
    //             h = NULL;
    //         }
    //     }
    // }

    void aprilTagThread()
    {
        std::this_thread::sleep_for(std::chrono::seconds(1)); // wait for the thread to start

        while (true)
        {
            auto scene = camera.getImage();
            april_tag_detector.detect(scene);
        }
    }

    void pause_logging_process()
    {
        push_back_encode = true; // Set push_back_encode to true when pausing
    }

    void resume_logging_process()
    {
        push_back_encode = false; // Reset push_back_encode to false when resuming
    }

    void start_logging_process()
    {
        log_process = std::thread(&LearningWalk::loggingProcess, this);
    }

    void stop_logging_process()
    {
        log_process.join();
    }

    void searchGoal()
    {
        std::thread april_tag_thread(&LearningWalk::aprilTagThread, this);
        start_logging_process();
        auto start = std::chrono::high_resolution_clock::now();

        while (true)
        {
            control.goStraight(1000);
            pause_logging_process();

            std::this_thread::sleep_for(std::chrono::seconds(5));
            resume_logging_process();
        }
    }

    void clear()
    {
        for (int i = 0; i < kinematic.left_encoder_tics.size(); i++)
        {
            std::vector<int> temp{kinematic.left_encoder_tics[i], kinematic.right_encoder_tics[i]};
            encoder_tics.push_back(temp);
        }

        encoder_values.clear();
        kinematic.left_encoder_tics.clear();
        kinematic.right_encoder_tics.clear();
        kinematic.time_point.clear();
    }
};