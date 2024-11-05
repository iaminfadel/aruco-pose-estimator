#ifndef ARUCO_POSE_PROCESSOR_H
#define ARUCO_POSE_PROCESSOR_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <yaml-cpp/yaml.h>

namespace aruco_pose_estimator
{
    class ArucoPoseProcessor
    {
    public:
        ArucoPoseProcessor();
        std::vector<std::vector<double>> processImage(const cv::Mat &input_image);
        void setCameraInfo(const sensor_msgs::CameraInfo &camera_info);
        cv::Mat debuggingImage;

    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        cv::Ptr<cv::aruco::Dictionary> aruco_dictionary_;
        cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_;

        std::string desired_dictionary_ = "DICT_5X5_100";
        float marker_size_ = 0.14; // meters

        cv::Mat drawAxis(cv::Mat &img, const std::vector<cv::Point2f> &corners,
                         const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                         const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                         float length = 0.1);

        std::vector<cv::Point3f> createBoardObjectPoints(float marker_size);
    };
}
#endif // ARUCO_POSE_PROCESSOR_H