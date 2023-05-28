#include <iostream>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam.h>

using namespace cv;
using namespace std;

class Camera
{
private:
    raspicam::RaspiCam camera;
    int raw_capture_len;
    unsigned char *raw_capture;

public:
    int width = 320;
    int height = 240;

    Camera()
    {
        setup(&camera);
        if (!camera.open()) // Is the camera open?
        {
            std::cerr << "Error opening camera." << std::endl;
        }
        std::cout << "Waiting for camera stabilisation...";
        usleep(3000000);
        std::cout << "done" << std::endl;
        raw_capture_len = camera.getImageTypeSize(raspicam::RASPICAM_FORMAT_RGB);
        raw_capture = new unsigned char[raw_capture_len];
    }

    void setup(raspicam::RaspiCam *camera)
    {
        camera->setFormat(raspicam::RASPICAM_FORMAT_RGB);

        // Set image resolution
        camera->setWidth(width);
        camera->setHeight(height); 

        // Flip camera image vertically and horizontally
        // because camera is mounted upside down
        camera->setVerticalFlip(true);   // true
        camera->setHorizontalFlip(true); // true
    }

    // Single image
    Mat getImage()
    {
        // capture frames from the camera
        camera.grab();
        camera.retrieve(raw_capture);
        cv::Mat image = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, raw_capture);

        // return the image
        return image;
    }

    // Video stream
    // Mainly used for testing framerate 
    void getVideo()
    {
        // create window to display the video
        cv::namedWindow("Frame", cv::WINDOW_AUTOSIZE);

        while (true)
        {
            // capture frame from the camera
            camera.grab();
            camera.retrieve(raw_capture);
            // create a Mat object from the raw_capture buffer
            cv::Mat image = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC3, raw_capture);
            // flip the image
            // cv::flip(image, frame, 0);
            // cv::flip(frame, frame, 1);
            // show the frame
            cv::imshow("Frame", image);

            // check for user input to quit
            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q')
            {
                break;
            }
        }
    }
};