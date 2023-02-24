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
#include <chrono>
#include <thread>
#include <stdio.h>
#include <stdlib.h>

#include <pigpio.h>

// run code with "sudo /home/pi/Master-Thesis/CPP/build/master" in the terminal
// If you have run sudo pigpiod in terminal, use "sudo killall pigpiod" before running this code

int main(int argc, char **argv)
{
	int time = 5000;

	int inti = gpioInitialise();

	std::cout << "Init: " << inti << std::endl;

	if (gpioInitialise() < 0)
	{
		std::cout << "Failed" << std::endl;
		// pigpio initialisation failed.
	}
	else
	{
		std::cout << "Succeded" << std::endl;
		// pigpio initialised okay.
	}

	// if (gpioInitialise()<0) exit(1);

	gpioSetPWMrange(12, 255); // Now 2000 is fully on
	gpioSetPWMrange(13, 255); // Now 2000 is fully on

	gpioSetPWMfrequency(12, 1000);
	gpioSetPWMfrequency(13, 1000);

	gpioSetMode(12, 1); // Set GPIO18 as output.
	gpioSetMode(13, 1); // Set GPIO18 as output.
	gpioSetMode(16, 1); // Set GPIO18 as output.
	gpioSetMode(26, 1); // Set GPIO18 as output.
	gpioSetMode(5, 1);	// Set GPIO18 as output.
	gpioSetMode(6, 1);	// Set GPIO18 as output.

	gpioWrite(16, 1); // Set GPIO24 high.
	gpioWrite(26, 0); // Set GPIO24 high.
	gpioWrite(5, 1);  // Set GPIO24 high.
	gpioWrite(6, 0);  // Set GPIO24 high.

	gpioPWM(12, 255); // Sets GPIO18 half on.
	gpioPWM(13, 255); // Sets GPIO18 half on.

	std::cout << "Before" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(time));
	std::cout << "After" << std::endl;

	gpioPWM(12, 0); // Sets GPIO18 half on.
	gpioPWM(13, 0); // Sets GPIO18 half on.

	gpioTerminate();

	return 0;
}

// #include "RPLidar/rplidar.h"
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

// 	rp.setBaudrate(1000000);
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