// run code with "sudo /home/pi/Master-Thesis/CPP/build/master" in the terminal
// This only works if using a Raspberry Pi and the folder structure is the same
// If different folder structure run the main as sudo 

// If you have run sudo pigpiod in terminal, use "sudo killall pigpiod" before running this code

#include <iostream>
#include "learningwalk.h"

// Initialize the learning walk class
// Run the test you want from learningwalk.h
int main() {
    LearningWalk walk;

    walk.searchGoal(); 

    return 0;
}
// To test the robot going straight: 
// make a simple method setting both motors pwm to the same and stop the robot occassionaly 




// Below code is used to test the LIDAR
// Did not get to making it as a class and use it in the learning walk behaviour

// #include <RPLidar/rplidar.h>
// #include <chrono>
// #include <iostream>
// #include <thread>

// int main(int argc, char *argv[])
// {
// 	std::cout << "RPLidar C++ Interface Demo" << std::endl;
// 	std::cout << "--------------------------" << std::endl;

// 	RPLidar rp("/dev/ttyUSB0");

// 	rp.startMotor();

// 	std::cout << "test" << std::endl;

// 	rp.setBaudrate(115200);
// 	rp.setMaxDistance(16.0);

// 	std::cout << "here" << std::endl;

// 	using namespace std::chrono;
// 	system_clock::time_point start = system_clock::now();

// 	std::cout << "Before" << std::endl;

// 	while (duration_cast<seconds>(system_clock::now() - start).count() < 1)
// 	{
// 		if (auto scan = rp.poll())
// 		{
// 			std::cout << *scan << std::endl;
// 		}
// 		else
// 		{
// 			std::this_thread::sleep_for(milliseconds(10));
// 		}
// 	}

// 	std::cout << "After" << std::endl;

//     rp.stopMotor();

//     std::this_thread::sleep_for(milliseconds(5000));

// 	return 0;
// }