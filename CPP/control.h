#include <pigpio.h>
#include <time.h>
#include <cmath>
#include <vector>
#include <chrono>

class Controller
{
private:
    int frequency = 1000;
    int range = 255;
    int pins[10] = {5, 6, 12, 13, 16, 26, 17, 18, 22, 23};
    int n = 6;
    std::vector<std::vector<int>> datalog;
    double timer = time_time();
    int tics_to_rotate = 0;

public:
    int minSpeed = 200;
    int maxSpeed = 255;
    int speed = 200;

    Controller()
    {
        if (gpioInitialise() < 0)
        {
            std::cerr << "pigpio initialisation failed." << std::endl;
            return;
        }

        gpioSetPWMrange(pins[2], range);
        gpioSetPWMrange(pins[3], range);

        gpioSetPWMfrequency(pins[2], frequency);
        gpioSetPWMfrequency(pins[3], frequency);
    }

    ~Controller()
    {
        gpioTerminate();
    }

    double setTicsToRotate(int pwm)
    {
        return -0.74 * pwm + 600.15;
    }

    double setTics180(int pwm)
    {
        return -0.36 * pwm + 757.73;
    }

    void initGPIOPins()
    {
        for (int i = 0; i < n; i++)
        {
            gpioSetMode(pins[i], PI_OUTPUT);
        }
    }

    void setLeftMotor(int speed, int dir)
    {
        if (dir > 0)
        {
            gpioWrite(pins[4], 1);
            gpioWrite(pins[5], 0);
        }
        if (dir <= 0)
        {
            gpioWrite(pins[4], 0);
            gpioWrite(pins[5], 1);
        }
        if (speed > maxSpeed)
        {
            speed = maxSpeed;
        }
        if (speed < 0)
        {
            speed = 0;
        }
        gpioPWM(pins[2], speed);
    }

    void setRightMotor(int speed, int dir)
    {
        if (dir > 0)
        {
            gpioWrite(pins[0], 1);
            gpioWrite(pins[1], 0);
        }
        if (dir <= 0)
        {
            gpioWrite(pins[0], 0);
            gpioWrite(pins[1], 1);
        }
        if (speed > maxSpeed)
        {
            speed = maxSpeed;
        }
        if (speed < 0)
        {
            speed = 0;
        }
        gpioPWM(pins[3], speed);
    }

    std::vector<int> get_encode_values()
    {
        int left_A = gpioRead(pins[8]);
        int left_B = gpioRead(pins[9]);
        int right_A = gpioRead(pins[6]);
        int right_B = gpioRead(pins[7]);

        std::vector<int> temp;
        temp.push_back(left_A);
        temp.push_back(left_B);
        temp.push_back(right_A);
        temp.push_back(right_B);

        return temp;
    }

    void setMotorSpeedDirection(int SpeedL, int SpeedR, int dirL, int dirR, bool log)
    {
        setLeftMotor(SpeedL, dirL);
        setRightMotor(SpeedR, dirR);

        if (log == true)
        {
            logging(SpeedL, SpeedR, dirL, dirR);
        }
    }

    void logging(int SpeedL, int SpeedR, int dirL, int dirR)
    {
        std::vector<int> temp;
        temp.push_back(SpeedL);
        temp.push_back(SpeedR);
        temp.push_back(dirL);
        temp.push_back(dirR);

        if (datalog.empty())
        {
            timer = time_time();
        }
        else
        {
            double end = timer;
            timer = time_time();
            double time_elapsed = timer - end;
            datalog.back().push_back(time_elapsed);
        }

        datalog.push_back(temp);
    }

    std::vector<std::vector<int>> get_logging()
    {
        return datalog;
    }

    void goStraight(int tics_to_go)
    {
        int tics_l = 0;
        int tics_r = 0;
        float pwm_factor = 1.0; // This value need to be adjusted and tested
        std::vector<int> current_encoder = get_encode_values();

        while (tics_l <= tics_to_go && tics_r <= tics_to_go)
        {
            std::vector<int> temp = get_encode_values();

            if (temp[0] != current_encoder[0] || temp[1] != current_encoder[1])
            {
                tics_l += 1;
            }

            if (temp[2] != current_encoder[2] || temp[3] != current_encoder[3])
            {
                tics_r += 1;
            }

            current_encoder = temp;

            if (tics_r <= tics_to_go)
            {
                int diff_r = 0;

                if (tics_r < tics_l)
                {
                    diff_r = (tics_l - tics_r) * pwm_factor;
                }

                setRightMotor(speed + diff_r, 1);
            }
            else
            {
                setRightMotor(0, 1);
                setLeftMotor(0, 1);
                tics_l = tics_to_go + 1;
            }

            if (tics_l <= tics_to_go)
            {
                int diff_l = 0;

                if (tics_l < tics_r)
                {
                    diff_l = (tics_r - tics_l) * pwm_factor;
                }

                setLeftMotor(speed + diff_l, 1);
            }
            else
            {
                setRightMotor(0, 1);
                setLeftMotor(0, 1);
                tics_r = tics_to_go + 1;
            }
        }
    }

    void turn(double theta)
    {
        int tics_l = 0;
        int tics_r = 0;
        int tics_turn = setTics180(speed);

        tics_to_rotate = (tics_turn / M_PI) * (fabs(theta));
        // std::cout << "Tics to rotate: " << tics_to_rotate << std::endl;

        auto current_encoder = get_encode_values();

        while (tics_l <= tics_to_rotate && tics_r <= tics_to_rotate)
        {
            auto temp = get_encode_values();

            if (temp[0] != current_encoder[0] || temp[1] != current_encoder[1])
            {
                tics_l += 1;
            }

            if (temp[2] != current_encoder[2] || temp[3] != current_encoder[3])
            {
                tics_r += 1;
            }

            current_encoder = temp;

            if (theta < 0)
            {
                if (tics_r <= tics_to_rotate)
                {
                    setRightMotor(speed, 1);
                }
                if (tics_l <= tics_to_rotate)
                {
                    setLeftMotor(speed, 0);
                }
                if (tics_r > tics_to_rotate || tics_l > tics_to_rotate)
                {
                    setRightMotor(0, 1);
                    setLeftMotor(0, 0);
                }
            }
            else
            {
                if (tics_r <= tics_to_rotate)
                {
                    setRightMotor(speed, 0);
                }
                if (tics_l <= tics_to_rotate)
                {
                    setLeftMotor(speed, 1);
                }
                if (tics_r > tics_to_rotate || tics_l > tics_to_rotate)
                {
                    setRightMotor(0, 0);
                    setLeftMotor(0, 1);
                }
            }
        }
    }

    std::vector<double> parameters()
    {
        std::vector<double> param;
        param.push_back(maxSpeed);
        param.push_back(minSpeed);
        param.push_back(speed);
        int tics_turn = setTics180(speed);
        param.push_back(tics_turn);

        return param;
    }

    void stopMotor()
    {
        setLeftMotor(0, 0);
        setRightMotor(0, 0);
        gpioTerminate();
    }
};