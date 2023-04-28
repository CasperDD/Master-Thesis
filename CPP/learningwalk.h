#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <thread>
#include <future>
#include <chrono>
#include <numeric>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <fstream>

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

    // int head = 1;
    // int tail = 1;
    // int temp = 0;

    int center = camera.width / 2;
    int w;
    int h;
    int x;
    bool paused;

    std::atomic<bool> push_back_encode{false};
    std::atomic<bool> turn_sig{false};
    std::atomic<bool> log_thread{false};
    std::atomic<bool> apriltag_thread{false};

    bool goal_found = false;
    bool in_center = false;
    bool pause_log = false;

    float learn_rate;
    float pred_weight;
    float reflex_weight;
    float reflex_old;
    float ico_out;

    std::thread log_process;

    std::mutex mtx;
    std::condition_variable cova;

    std::vector<double> theta_file;
    std::vector<double> periodic_distances;
    std::vector<std::vector<double>> timepoint_file;

    std::vector<std::vector<int>> encoder_values;
    std::vector<std::vector<int>> encoder_tics;
    std::vector<std::vector<std::vector<int>>> encoder_tics_file;

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
        int head = 1, tail = 1, temp = 0;
        auto timer = std::chrono::high_resolution_clock::now(); // Create timer to the current instance
        while (log_thread == true)
        {
            // Check if logging is paused
            while (pause_log)
            {
                // std::cout << "Paused" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            // Log encoder values
            // std::cout << "Logging" << std::endl;
            encoder_values.emplace_back(control.get_encode_values());

            if (push_back_encode)
            {
                // std::cout << "temp: " << temp << std::endl;
                encoder_values.erase(encoder_values.begin(), encoder_values.begin() + temp);
                head = encoder_values.size();
                temp = head;
                // std::cout << "tail: " << tail << std::endl;
                // std::cout << "head: " << head << std::endl;
                std::chrono::_V2::high_resolution_clock::time_point end = timer;
                timer = std::chrono::high_resolution_clock::now();
                double time_elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(timer - end).count();
                kinematic.time_point.push_back(time_elapsed);

                std::vector<int> temp = ticCount(tail, head);
                std::cout << "left: " << temp.at(0) * -1 << std::endl;
                std::cout << "right: " << temp.at(1) << std::endl;

                kinematic.left_encoder_tics.push_back(temp.at(0) * -1);
                kinematic.right_encoder_tics.push_back(temp.at(1));

                kinematic.left_tics += (temp.at(0) * -1);
                kinematic.right_tics += temp.at(1);

                std::cout << "sum left: " << kinematic.left_tics << std::endl;
                std::cout << "sum right: " << kinematic.right_tics << std::endl;

                push_back_encode = false;
            }
        }
        encoder_values.clear();
        // std::cout << "Exitted out of the logging thread" << std::endl;
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

        // std::cout << "dir vec: " << std::endl;
        // for (int i; i < dir_vec.size(); i++)
        // {
        //     std::cout << dir_vec.at(i) << " ";
        // }
        periodic_distances.push_back(dir_vec[0]);
        theta_file.push_back(dir_vec[1]);

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

        while (apriltag_thread == true)
        {
            auto scene = camera.getImage();
            april_tag_detector.detect(scene);
        }
    }

    void pause_logging_process()
    {
        pause_log = true; // Set pause_log to true when pausing
        push_back_encode = true;
    }

    void resume_logging_process()
    {
        pause_log = false; // Reset pause_log to false when resuming
    }

    void start_logging_process()
    {
        log_thread = true;
        log_process = std::thread(&LearningWalk::loggingProcess, this);
    }

    void stop_logging_process()
    {
        log_thread = false;
        log_process.join();
    }

    void reset()
    {
        int head = 1;
        int tail = 1;
        int temp = 0;
        encoder_values.clear();

        // std::cout << "Reset variables: " << head << ", " << tail  << ", " << temp << std::endl;
    }

    void file(std::string file_name)
    {
        std::ofstream my_file(file_name);
        std::vector<double> param = control.parameters();

        my_file << "MaxSpeed: " << param.at(0) << "\n";
        my_file << "MinSpeed: " << param.at(1) << "\n";
        my_file << "Actual speed: " << param.at(2) << "\n";
        my_file << "turn_tic: " << param.at(3) << "\n";

        my_file << "\n";
        my_file << "\n";
        my_file << "\n";

        my_file << "Theta (how much the robot turn): "
                << "\n";
        for (int i = 0; i < theta_file.size(); i++)
        {
            my_file << theta_file.at(i) << "\n";
        }

        my_file << "\n";
        my_file << "\n";
        my_file << "\n";

        my_file << "Distance from point x to home: "
                << "\n";
        for (int i = 0; i < periodic_distances.size(); i++)
        {
            my_file << periodic_distances.at(i) << "\n";
        }
    }

    void file_variables()
    {
        for (int i = 0; i < kinematic.left_encoder_tics.size(); i++)
        {
            std::vector<int> temp;
            temp.push_back(kinematic.left_encoder_tics.at(i));
            temp.push_back(kinematic.right_encoder_tics.at(i));
            encoder_tics.push_back(temp);
        }
        encoder_tics_file.push_back(encoder_tics);
        timepoint_file.push_back(kinematic.time_point);
    }

    void searchGoal()
    {
        int i = 0;
        // push_back_encode.store(false);
        apriltag_thread = true;
        std::thread april_tag_thread = std::thread(&LearningWalk::aprilTagThread, this);
        // start_logging_process();
        auto start = std::chrono::high_resolution_clock::now();

        while (i < 4)
        {
            // std::cout << "start of while loop" << std::endl;
            start_logging_process();
            for (int j = 0; j < 4; j++) // j < 4, since it takes 4 runs to get accross the arena
            {
                // std::cout << "in for loop: " << j << std::endl;
                control.goStraight(1000);
                pause_logging_process();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                double turn = updateDirVec();
                resume_logging_process();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                control.turn(turn);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                control.turn(-turn);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            stop_logging_process();
            std::cout << "Iteration of while loop: " << i << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(20));
            // reset();
            // std::this_thread::sleep_for(std::chrono::seconds(10));
            // std::cout << "Iteration of while loop: " << i << std::endl;
            i++;
        }
        // std::cout << "exitted searchGoal" << std::endl;
        // stop_logging_process();
        apriltag_thread = false;
        april_tag_thread.join();
        // std::cout << "after ending april tag thread" << std::endl;
        file("/home/pi/Master-Thesis/CPP/control_data5.txt");
    }
};