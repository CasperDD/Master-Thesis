// #include <ctime>
// #include <iostream>
// #include <raspicam/raspicam_cv.h>
// using namespace std;

// int main ( int argc,char **argv ) {

// 	time_t timer_begin,timer_end;
// 	raspicam::RaspiCam_Cv Camera;
// 	cv::Mat image;
// 	int nCount=100;
// 	//set camera params
// 	Camera.set( cv::CAP_PROP_FORMAT, CV_8UC1 );
// 	//Open camera
// 	cout<<"Opening Camera..."<<endl;
// 	if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
// 	//Start capture
// 	cout<<"Capturing "<<nCount<<" frames ...."<<endl;
// 	time ( &timer_begin );
// 	for ( int i=0; i<nCount; i++ ) {
// 		Camera.grab();
// 		Camera.retrieve ( image);
// 		if ( i%5==0 )  cout<<"\r captured "<<i<<" images"<<std::flush;
// 	}
// 	cout<<"Stop camera..."<<endl;
// 	Camera.release();
// 	//show time statistics
// 	time ( &timer_end ); /* get current time; same as: timer = time(NULL)  */
// 	double secondsElapsed = difftime ( timer_end,timer_begin );
// 	cout<< secondsElapsed<<" seconds for "<< nCount<<"  frames : FPS = "<<  ( float ) ( ( float ) ( nCount ) /secondsElapsed ) <<endl;
// 	//save image
// 	cv::imwrite("../raspicam_cv_image.jpg",image);
// 	cout<<"Image saved at raspicam_cv_image.jpg"<<endl;
// }






#include <iostream>
#include "learningwalk.h"

int main() {
    LearningWalk walk;

    walk.TurnTest();

    return 0;
}






// #include <iostream>
// #include "camera.h"
// #include "myapriltag.h"
// #include "control.h"
// #include "kinematics.h"

// using namespace std;

// int main()
// {
//     Camera camera;
//     AprilTagDetector detector;
//     // Controller control;
//     // Kinematics kinematics;

//     // control.goStraight(2000);

//     // waitKey(1000);

//     // control.turn(10);

//     // waitKey(1000);

//     // control.stopMotor();

//     while (true)
//     {
//         Mat image = camera.getImage();
//         std::vector<double> apriltags = detector.detect(image);
//         std::cout << "Center point: (" << apriltags[0] << ", " << apriltags[1] << ", " << apriltags[2] << ")" << std::endl;
//         // detector.display_window(image);
//     }
//     return 0;
// }







// #include <pigpio.h>
// #include <chrono>
// #include <iostream>
// #include <tuple>
// #include <thread>
// // #include "control.h"

// int main() {
//     gpioInitialise();
//     // Control control;

//     int frequency = 1000;
//     int range = 255;
//     int pins[] = {5, 6, 12, 13, 16, 26, 17, 18, 22, 23};

//     gpioSetPWMrange(pins[2], range);
//     gpioSetPWMrange(pins[3], range);
//     gpioSetPWMfrequency(pins[2], frequency);
//     gpioSetPWMfrequency(pins[3], frequency);

//     int count = 0;
//     int right_A = gpioRead(pins[8]);
//     int right_B = gpioRead(pins[9]);

//     std::tuple<int, int> prev_state(right_A, right_B);
//     gpioPWM(12, 200); // Sets GPIO18 half on.
//     gpioPWM(13, 200); // Sets GPIO18 half on.

//     auto start = std::chrono::high_resolution_clock::now();
//     double time_elapsed = 0;

//     while (time_elapsed < 5.8) { //true
//         int right_A_prev = right_A;
//         int right_B_prev = right_B;
//         int left_A = gpioRead(pins[8]);
//         int left_B = gpioRead(pins[9]);
//         right_A = gpioRead(pins[6]);
//         right_B = gpioRead(pins[7]);

//         std::tuple<int, int> curr_state(right_A, right_B); //, left_A, left_B
//         std::cout << std::get<0>(curr_state) << ", " << std::get<1>(curr_state) << std::endl;
//         if (prev_state == std::make_tuple(0, 0)) {
//             if (curr_state == std::make_tuple(0, 1)) {
//                 count += 1;
//             } else if (curr_state == std::make_tuple(1, 0)) {
//                 count -= 1;
//             }
//         } else if (prev_state == std::make_tuple(0, 1)) {
//             if (curr_state == std::make_tuple(0, 0)) {
//                 count -= 1;
//             } else if (curr_state == std::make_tuple(1, 1)) {
//                 count += 1;
//             }
//         } else if (prev_state == std::make_tuple(1, 0)) {
//             if (curr_state == std::make_tuple(1, 1)) {
//                 count -= 1;
//             } else if (curr_state == std::make_tuple(0, 0)) {
//                 count += 1;
//             }
//         } else if (prev_state == std::make_tuple(1, 1)) {
//             if (curr_state == std::make_tuple(1, 0)) {
//                 count += 1;
//             } else if (curr_state == std::make_tuple(0, 1)) {
//                 count -= 1;
//             }
//         }
//         prev_state = curr_state;

//         auto end = std::chrono::high_resolution_clock::now();
//         time_elapsed = std::chrono::duration<double>(end - start).count();
//         // std::cout << "Time elapsed: " << time_elapsed << std::endl;
//         // std::cout << "Count: " << count << std::endl;
//         // std::this_thread::sleep_for(std::chrono::milliseconds(1));
//     }
//     // control.setLeftMotor(0, 1);
//     // control.setRightMotor(0, 1);
//     gpioPWM(12, 0); // Sets GPIO18 half on.
//     gpioPWM(13, 0); // Sets GPIO18 half on.
//     gpioTerminate();
//     return 0;
// }

// #include <pigpio.h>
// #include <chrono>
// #include <iostream>
// #include <thread>
// #include <utility>
// #include <iomanip> // for std::fixed and std::setprecision
// // #include "control.h" // assuming 'control' class is defined in a separate header file

// int main()
// {
//     if (gpioInitialise() < 0)
//     {
//         // pigpio initialization failed
//         return 1;
//     }

//     int pins[] = {5, 6, 12, 13, 16, 26, 17, 18, 22, 23};
//     int frequency = 1000;
//     int range = 255;

//     gpioSetPWMrange(pins[2], range);
//     gpioSetPWMrange(pins[3], range);

//     gpioSetPWMfrequency(pins[2], frequency);
//     gpioSetPWMfrequency(pins[3], frequency);

//     int count = 0;

//     int right_A = gpioRead(pins[8]);
//     int right_B = gpioRead(pins[9]);
// 	int left_A = gpioRead(pins[8]);
// 	int left_B = gpioRead(pins[9]);
//     int prev_state = right_A << 1 | right_B;

//     gpioPWM(12, 200); // Sets GPIO18 half on.
//     gpioPWM(13, 200); // Sets GPIO18 half on.

//     auto start = std::chrono::steady_clock::now();
//     double time_elapsed = 0;

//     while (true) //time_elapsed < 2.13
//     {
//         start = std::chrono::steady_clock::now();
//         int right_A_prev = right_A;
//         int right_B_prev = right_B;
//         left_A = gpioRead(pins[8]);
//         left_B = gpioRead(pins[9]);
//         right_A = gpioRead(pins[6]);
//         right_B = gpioRead(pins[7]);

// 		// std::cout << "Curr State: " << left_A << " " << left_B << " " << right_A << " " << right_B << std::endl;

//         int curr_state = right_A << 3 | right_B << 2 | left_A << 1 | left_B;
// 		// std::cout << curr_state << std::endl;

//         if (prev_state == std::make_pair(0, 0)) {
//             if (curr_state == std::make_pair(0, 1)) {
//                 count += 1;
//             }
//             else if (curr_state == std::make_pair(1, 0)) {
//                 count -= 1;
//             }
//         }
//         else if (prev_state == std::make_pair(0, 1)) {
//             if (curr_state == std::make_pair(0, 0)) {
//                 count -= 1;
//             }
//             else if (curr_state == std::make_pair(1, 1)) {
//                 count += 1;
//             }
//         }
//         else if (prev_state == std::make_pair(1, 0)) {
//             if (curr_state == std::make_pair(1, 1)) {
//                 count -= 1;
//             }
//             else if (curr_state == std::make_pair(0, 0)) {
//                 count += 1;
//             }
//         }
//         else if (prev_state == std::make_pair(1, 1)) {
//             if (curr_state == std::make_pair(1, 0)) {
//                 count += 1;
//             }
//             else if (curr_state == std::make_pair(0, 1)) {
//                 count -= 1;
//             }
// }

//         // Print curr_state
//         // std::cout << "Curr State: " << curr_state << std::endl;

//         // Perform desired operations with curr_state

//         prev_state = curr_state;

//         auto end = std::chrono::steady_clock::now();
//         std::chrono::duration<double> elapsed_seconds = end - start;
//         time_elapsed = elapsed_seconds.count();
//         // std::cout << "Time elapsed: " << std::fixed << std::setprecision(10) << time_elapsed << std::endl;
//     }

//     gpioTerminate(); // Terminate pigpio
//     return 0;
// }

// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <stdio.h>
// #include <stdlib.h>

// #include <pigpio.h>

// // run code with "sudo /home/pi/Master-Thesis/CPP/build/master" in the terminal
// // If you have run sudo pigpiod in terminal, use "sudo killall pigpiod" before running this code

// int main(int argc, char **argv)
// {
// 	int time = 5000;

// 	int inti = gpioInitialise();

// 	std::cout << "Init: " << inti << std::endl;

// 	if (gpioInitialise() < 0)
// 	{
// 		std::cout << "Failed" << std::endl;
// 		// pigpio initialisation failed.
// 	}
// 	else
// 	{
// 		std::cout << "Succeded" << std::endl;
// 		// pigpio initialised okay.
// 	}

// 	// if (gpioInitialise()<0) exit(1);

// 	gpioSetPWMrange(12, 255); // Now 2000 is fully on
// 	gpioSetPWMrange(13, 255); // Now 2000 is fully on

// 	gpioSetPWMfrequency(12, 1000);
// 	gpioSetPWMfrequency(13, 1000);

// 	gpioSetMode(12, 1); // Set GPIO18 as output.
// 	gpioSetMode(13, 1); // Set GPIO18 as output.
// 	gpioSetMode(16, 1); // Set GPIO18 as output.
// 	gpioSetMode(26, 1); // Set GPIO18 as output.
// 	gpioSetMode(5, 1);	// Set GPIO18 as output.
// 	gpioSetMode(6, 1);	// Set GPIO18 as output.

// 	gpioWrite(16, 1); // Set GPIO24 high.
// 	gpioWrite(26, 0); // Set GPIO24 high.
// 	gpioWrite(5, 1);  // Set GPIO24 high.
// 	gpioWrite(6, 0);  // Set GPIO24 high.

// 	gpioPWM(12, 255); // Sets GPIO18 half on.
// 	gpioPWM(13, 255); // Sets GPIO18 half on.

// 	std::cout << "Before" << std::endl;
// 	std::this_thread::sleep_for(std::chrono::milliseconds(time));
// 	std::cout << "After" << std::endl;

// 	gpioPWM(12, 0); // Sets GPIO18 half on.
// 	gpioPWM(13, 0); // Sets GPIO18 half on.

// 	gpioTerminate();

// 	return 0;
// }





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