#include <iostream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <unistd.h>
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

    int center = camera.width / 2;
    int w;
    int h;
    int x;
    bool paused;

    std::atomic<bool> push_back_encode{false};
    std::atomic<bool> turn_sig{false};
    std::atomic<bool> log_thread{false};
    std::atomic<bool> pirouette_check{false};
    std::atomic<bool> apriltag_thread{false};

    bool goal_found = false;
    bool in_center = false;
    bool pause_log = false;

    double learn_rate = 0.01; //Learning rate for the ICO learning 
    double pred_weight = 0.0;
    double reflex_weight = 1.0; 
    double reflex_old = 0.0;
    double ico_out = 0.0;
    double predict = 0.0;

    std::vector<double> predict_file;
    std::vector<double> reflex_file;
    std::vector<double> predict_weight_file;
    std::vector<double> ico_out_file;

    double radians_turned = 0.0;

    std::thread log_process;

    std::vector<double> vis_xy_id;
    std::vector<double> theta_file;      // Direction vector theta prediction the robot needs to turn
    std::vector<double> turn_theta_file; // How much the robot turned
    std::vector<double> periodic_distances;
    std::vector<std::vector<double>> timepoint_file;

    std::vector<std::vector<int>> encoder_values;
    std::vector<std::vector<int>> encoder_tics;
    std::vector<std::vector<std::vector<int>>> encoder_tics_file;

    std::vector<int> left_tics_file;
    std::vector<int> right_tics_file;

public:
    LearningWalk()
    {
        gsl_vector_set_zero(kinematic.X_Y_Theta);
    }

    ~LearningWalk() {}

    // Method used to count tics
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

    // Method used to figure out the amount of tics on each wheel
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

    // A process running parallel with the main code
    // It reads the encoder values and saves them in a vector while in motion
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
                head = encoder_values.size();
                temp = head;
                // std::cout << "tail: " << tail << std::endl;
                // std::cout << "head: " << head << std::endl;
                std::chrono::_V2::high_resolution_clock::time_point end = timer;
                timer = std::chrono::high_resolution_clock::now();
                double time_elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(timer - end).count();
                kinematic.time_point.push_back(time_elapsed);

                std::vector<int> temp = ticCount(tail, head);
                // std::cout << "left: " << temp.at(0) * -1 << std::endl;
                // std::cout << "right: " << temp.at(1) << std::endl;

                if (pirouette_check == true)
                {
                    kinematic.pirouette_left += (temp.at(0) * -1);
                    kinematic.pirouette_right += temp.at(1);
                }
                else
                {
                    kinematic.left_encoder_tics.push_back(temp.at(0) * -1);
                    kinematic.right_encoder_tics.push_back(temp.at(1));

                    kinematic.left_tics += (temp.at(0) * -1);
                    kinematic.right_tics += temp.at(1);

                    // std::cout << "sum left: " << kinematic.left_tics << std::endl;
                    // std::cout << "sum right: " << kinematic.right_tics << std::endl;
                }

                // std::cout << "left and right encoder tics: " << std::endl;
                // for (int i = 0; i < kinematic.left_encoder_tics.size(); i++)
                // {
                //     std::cout << kinematic.left_encoder_tics.at(i) << ", ";
                // }

                // std::cout << std::endl;

                // for (int i = 0; i < kinematic.right_encoder_tics.size(); i++)
                // {
                //     std::cout << kinematic.right_encoder_tics.at(i) << ", ";
                // }
                // std::cout << std::endl;

                push_back_encode = false;
                encoder_values.clear();
            }
        }

        // std::cout << "Exitted out of the logging thread" << std::endl;
    }

    // Logic for the ICO learning algorithm
    double ICO(double predict, double reflex)
    {
        double deri_reflex = reflex - reflex_old;

        pred_weight += learn_rate * predict * deri_reflex;

        double output = pred_weight * predict + reflex_weight * reflex;

        // Below print statements used to monitor the robot during runs
        // Can comment out if not needed
        std::cout << "Prediction weight: " << pred_weight << std::endl;

        std::cout << "ICO output: " << output << std::endl;

        std::cout << "Reflex old: " << reflex_old << "  reflex: " << reflex << std::endl;

        std::cout << "derivative reflex: " << deri_reflex << std::endl;

        predict_file.push_back(predict);
        reflex_file.push_back(reflex);
        predict_weight_file.push_back(pred_weight);
        ico_out_file.push_back(output);

        reflex_old = reflex;

        return output;
    }

    // Calls kinematic.h to update the direction vector to figure out the prediction 
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

        // cout << "Theta: " << theta << endl;

        return theta;
    }

    // Used to adjust the robot so that it faces the center of home
    // This makes the reflex signal
    void visionTurn(double x, double theta, int id)
    {
        if (x == NULL || id != 1)
        {
            if (theta < 0 && theta > -M_PI)
            {
                control.setRightMotor(control.speed, 1);
                control.setLeftMotor(control.speed, 0);
            }
            else if (theta < 0 && theta < -M_PI)
            {
                control.setRightMotor(control.speed, 0);
                control.setLeftMotor(control.speed, 1);
            }
            else if (theta > 0 && theta > M_PI)
            {
                control.setRightMotor(control.speed, 1);
                control.setLeftMotor(control.speed, 0);
            }
            else if (theta > 0 && theta < M_PI)
            {
                control.setRightMotor(control.speed, 0);
                control.setLeftMotor(control.speed, 1);
            }
            in_center = false;
        }
        else if (x < center - 10 && id == 1)
        {
            control.setLeftMotor(control.speed, 0);
            control.setRightMotor(control.speed, 1);
            in_center = false;
        }
        else if (x > center + 10 && id == 1)
        {
            control.setLeftMotor(control.speed, 1);
            control.setRightMotor(control.speed, 0);
            in_center = false;
        }
        else
        {
            control.setLeftMotor(0, 1);
            control.setRightMotor(0, 1);
            in_center = true;
        }
    }

    // Pause the logging process
    void pause_logging_process()
    {
        push_back_encode = true;
        pause_log = true; // Set pause_log to true when pausing
    }

    // Resume the logging process
    void resume_logging_process()
    {
        pause_log = false; // Reset pause_log to false when resuming
    }

    // Start the logging process
    void start_logging_process()
    {
        log_thread = true;
        log_process = std::thread(&LearningWalk::loggingProcess, this);
    }

    // Stop the logging process
    void stop_logging_process()
    {
        log_thread = false;
        log_process.join();
    }

    // Method used to save data to a file
    void file(std::string file_name)
    {
        std::ofstream my_file(file_name);
        std::vector<double> param = control.parameters();

        my_file << "MaxSpeed: " << param.at(0) << "\n";
        my_file << "MinSpeed: " << param.at(1) << "\n";
        my_file << "Actual speed: " << param.at(2) << "\n";
        my_file << "turn_tic: " << param.at(3) << "\n";

        my_file << "\n"
                << "\n";

        my_file << "Theta (what is sent that the robot should turn): "
                << "\n";
        for (int i = 0; i < theta_file.size(); i++)
        {
            my_file << theta_file.at(i) << "\n";
        }

        my_file << "\n"
                << "\n";

        my_file << "Radians turned (how much the robot actually turned): "
                << "\n";
        for (int i = 0; i < turn_theta_file.size(); i++)
        {
            my_file << turn_theta_file.at(i) << "\n";
        }

        my_file << "\n"
                << "\n";

        my_file << "Distance from point x to home: "
                << "\n";
        for (int i = 0; i < periodic_distances.size(); i++)
        {
            my_file << periodic_distances.at(i) << "\n";
        }

        // If ICO information is not needed comment out the rest of the method

        my_file << "\n"
                << "\n"
                << "\n";

        my_file << "ICO information: "
                << "\n";

        my_file << "Learning rate: " << learn_rate << "\n";

        my_file << "\n"
                << "\n";

        my_file << "Predict: "
                << "\n";
        for (int i = 0; i < predict_file.size(); i++)
        {
            my_file << predict_file.at(i) << "\n";
        }

        my_file << "\n"
                << "\n";

        my_file << "Reflex: "
                << "\n";
        for (int i = 0; i < reflex_file.size(); i++)
        {
            my_file << reflex_file.at(i) << "\n";
        }

        my_file << "\n"
                << "\n";

        my_file << "Prediction weight: "
                << "\n";
        for (int i = 0; i < predict_weight_file.size(); i++)
        {
            my_file << predict_weight_file.at(i) << "\n";
        }

        my_file << "\n"
                << "\n";

        my_file << "ICO output: "
                << "\n";
        for (int i = 0; i < ico_out_file.size(); i++)
        {
            my_file << ico_out_file.at(i) << "\n";
        }
    }

    // Method used for smaller tests to save data to a file
    void smallFile(std::string file_name)
    {
        std::ofstream my_file(file_name);
        std::vector<double> param = control.parameters();

        my_file << "MaxSpeed: " << param.at(0) << "\n";
        my_file << "MinSpeed: " << param.at(1) << "\n";
        my_file << "Actual speed: " << param.at(2) << "\n";
        my_file << "turn_tic: " << param.at(3) << "\n";

        my_file << "\n"
                << "\n";
        
        my_file << "Left tics: "
                << "\n";
        for (int i = 0; i < left_tics_file.size(); i++)
        {
            my_file << left_tics_file.at(i) << "\n";
        }

        my_file << "Right tics: "
                << "\n";
        for (int i = 0; i < right_tics_file.size(); i++)
        {
            my_file << right_tics_file.at(i) << "\n";
        }


    }

    // Method used to make calculate how much the robot have turned in place
    double turnInPlace(int tics_turned)
    {
        int half_turn = control.setTics180(control.speed);
        double theta = (tics_turned * M_PI) / half_turn;

        return theta;
    }

    // Method to perform the pirouette behaviour
    // All sleep(3) is needed to make sure the logging process is cought up with the encoder readings
    void pirouette(double predict)
    {
        resume_logging_process();
        pirouette_check = true;
        sleep(3);
        control.turn(predict);
        push_back_encode = true;
        sleep(3);
        pause_logging_process();
        int prediction_left = kinematic.pirouette_left;
        int prediction_right = kinematic.pirouette_right;

        int highest = std::max(abs(prediction_left), abs(prediction_right));

        double turned_predict = turnInPlace(highest);

        std::cout << "Actually turned on predict: " << turned_predict << std::endl;

        radians_turned = turnInPlace(highest);
        turn_theta_file.push_back(radians_turned);

        resume_logging_process();
        pirouette_check = true;
        sleep(3);

        while (in_center == false)
        {
            auto scene = camera.getImage();
            vis_xy_id = april_tag_detector.detect(scene);
            double x = vis_xy_id[0];
            visionTurn(x, predict, vis_xy_id[2]);
        }

        push_back_encode = true;
        sleep(3);
        pause_logging_process();

        int visTurnHighest = std::max(abs(kinematic.pirouette_left), abs(kinematic.pirouette_right));
        double withVisionTurn = turnInPlace(visTurnHighest);

        std::cout << "Amount turned with visionTurn: " << withVisionTurn << std::endl;

        double reflex = abs(withVisionTurn) - abs(predict);
        std::cout << "Reflex: " << reflex << std::endl;

        ico_out = ICO(predict, reflex);

        kinematic.pirouette_right = 0;
        kinematic.pirouette_left = 0;

        resume_logging_process();
        sleep(3);
        control.turn(withVisionTurn);
        push_back_encode = true;
        sleep(3);
        pause_logging_process();

        int highest_2 = std::max(abs(kinematic.pirouette_left), abs(kinematic.pirouette_right));

        double turned_back = turnInPlace(highest_2);

        std::cout << "Amount turned back: " << turned_back << std::endl;

        pirouette_check = false;
        in_center = false;
        kinematic.pirouette_right = 0;
        kinematic.pirouette_left = 0;
    }

    // Method for control data
    void controlTurn(double theta)
    {
        resume_logging_process();
        pirouette_check = true;
        sleep(3);
        control.turn(theta);
        push_back_encode = true;
        sleep(3);
        pause_logging_process();

        std::cout << "Pirouette left wheel: " << kinematic.pirouette_left << std::endl;
        std::cout << "Pirouette right wheel: " << kinematic.pirouette_right << std::endl;

        int highest = std::max(abs(kinematic.pirouette_left), abs(kinematic.pirouette_right));

        radians_turned = turnInPlace(highest);
        turn_theta_file.push_back(radians_turned);
        std::cout << "radians_turned: " << radians_turned << std::endl;

        control.turn(-theta);
        pirouette_check = false;
        kinematic.pirouette_right = 0;
        kinematic.pirouette_left = 0;
        sleep(3);
    }

    // Method used for learning walk
    // Currently only going straight on each wheel before performing a pirouette
    void searchGoal()
    {
        int i = 1;
        bool while_loop = true;
        start_logging_process();
        int distance = 500; // How many tics the robot will go before stopping

        auto scene = camera.getImage();
        vis_xy_id = april_tag_detector.detect(scene);
        control.setMotorSpeedDirection(0, 0, 1, 1, false);
        push_back_encode = true;

        sleep(10); // Need the camera and logging process to startup

        while (while_loop == true)
        {
            scene = camera.getImage();
            vis_xy_id = april_tag_detector.detect(scene);

            control.setMotorSpeedDirection(control.speed, control.speed, 1, 1, false); // Can be changed to a search/exploration algorithm
            push_back_encode = true;

            if (kinematic.left_tics >= distance && kinematic.right_tics >= distance)
            {
                control.setMotorSpeedDirection(0, 0, 1, 1, false);
                push_back_encode = true;
                sleep(3);
                pause_logging_process();
                std::cout << "sum left: " << kinematic.left_tics << std::endl;
                std::cout << "sum right: " << kinematic.right_tics << std::endl;

                double theta_dirVec = updateDirVec();
                predict = abs(theta_dirVec) + ico_out;
                if (theta_dirVec < 0)
                {
                    predict = -predict;
                }
                std::cout << "Predict: " << predict << std::endl;

                pirouette(predict);

                kinematic.left_tics = 0;
                kinematic.right_tics = 0;

                std::cout << "iterator i: " << i << std::endl;

                if (i == 18)
                {
                    std::cout << "i equal 18" << std::endl;
                    while_loop = false;
                }
                // Do not need this if operating in an arena that is big enough for the robot to keep going
                else if (i % 6 == 0)
                {
                    std::cout << "20 sec pause" << std::endl;
                    sleep(20);
                }
                i++;

                resume_logging_process();

            }
        }

        // std::cout << "exitted searchGoal" << std::endl;
        stop_logging_process();

        // Rename the file or else the data will be overwritten
        file("/home/pi/Master-Thesis/CPP/Data/StraightICO/StraightICO_lr0.01_data5.txt");
    }

    // Method used to test the robot turning
    void TurnTest()
    {
        int i = 1;
        bool while_loop = true;

        start_logging_process();
        int distance = 500; // How many tics the robot will go before stopping

        auto scene = camera.getImage();
        vis_xy_id = april_tag_detector.detect(scene);
        control.setMotorSpeedDirection(0, 0, 1, 1, false);
        push_back_encode = true;

        sleep(10); // Need the camera and logging process to startup

        while (while_loop == true)
        {
            scene = camera.getImage();
            vis_xy_id = april_tag_detector.detect(scene);

            control.setMotorSpeedDirection(control.speed, control.speed, 0, 1, false); // Can be changed to a search/exploration algorithm
            push_back_encode = true;

            if (abs(kinematic.left_tics) >= distance && abs(kinematic.right_tics) >= distance)
            {
                control.setMotorSpeedDirection(0, 0, 1, 1, false);
                push_back_encode = true;

                sleep(3);
                pause_logging_process();

                std::cout << "sum left: " << kinematic.left_tics << std::endl;
                std::cout << "sum right: " << kinematic.right_tics << std::endl;

                left_tics_file.push_back(kinematic.left_tics);
                right_tics_file.push_back(kinematic.right_tics);

                kinematic.left_tics = 0;
                kinematic.right_tics = 0;

                bool inner_loop = true;

                resume_logging_process();

                while (inner_loop == true)
                {
                    scene = camera.getImage();
                    vis_xy_id = april_tag_detector.detect(scene);

                    control.setMotorSpeedDirection(control.speed, control.speed, 1, 0, false); // Can be changed to a search/exploration algorithm
                    push_back_encode = true;

                    if (abs(kinematic.left_tics) >= distance && abs(kinematic.right_tics) >= distance)
                    {
                        control.setMotorSpeedDirection(0, 0, 1, 1, false);
                        push_back_encode = true;

                        sleep(3);
                        pause_logging_process();

                        std::cout << "sum left: " << kinematic.left_tics << std::endl;
                        std::cout << "sum right: " << kinematic.right_tics << std::endl;

                        left_tics_file.push_back(kinematic.left_tics);
                        right_tics_file.push_back(kinematic.right_tics);

                        kinematic.left_tics = 0;
                        kinematic.right_tics = 0;

                        while_loop = false;
                        inner_loop = false;

                        resume_logging_process();
                    }
                }

            }
        }
        stop_logging_process();

        // Rename the file or else the data will be overwritten
        smallFile("/home/pi/Master-Thesis/CPP/Data/TurningAround/Turn_left_around_data5.txt");
    }
};