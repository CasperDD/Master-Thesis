#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>

class AprilTagDetector {
public:
    AprilTagDetector(int resize_width=320, int resize_height=240)
        : resize_width_(resize_width), resize_height_(resize_height) {
        detector_ = apriltag_detector_create();
        apriltag_family_t *tf = tagStandard41h12_create();
        apriltag_detector_add_family(detector_, tf);
    }

    ~AprilTagDetector() {
        apriltag_detector_destroy(detector_);
    }

    void detect(cv::Mat& image) {
        cv::Mat gray_image;
        cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
        image_u8_t im = { .width = gray_image.cols,
                          .height = gray_image.rows,
                          .stride = gray_image.cols,
                          .buf = gray_image.data
                        };
        zarray_t* detections = apriltag_detector_detect(detector_, &im);

        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* detection;
            zarray_get(detections, i, &detection);
            std::cout << "Detected AprilTag with ID " << detection->id << std::endl;
            // apriltag_detection_destroy(detection);
        }

        apriltag_detections_destroy(detections);

        cv::resize(image, image, cv::Size(resize_width_, resize_height_));
        cv::imshow("AprilTag", image);
        cv::waitKey(1);
    }

private:
    apriltag_detector_t* detector_;
    int resize_width_;
    int resize_height_;
};
