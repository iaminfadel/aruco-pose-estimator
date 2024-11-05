#include "aruco_pose_estimator/aruco_pose_processor.h"
#include <opencv2/calib3d.hpp>

namespace aruco_pose_estimator
{

    ArucoPoseProcessor::ArucoPoseProcessor()
    {
        // Define ArUco dictionary mapping
        std::map<std::string, int> ARUCO_DICT = {
            {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
            {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
            // Add other dictionaries as needed
        };

        // Load the desired ArUco dictionary
        int dict_id = ARUCO_DICT[desired_dictionary_];
        aruco_dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
        aruco_parameters_ = cv::aruco::DetectorParameters::create();
    }

    void ArucoPoseProcessor::setCameraInfo(const sensor_msgs::CameraInfo &camera_info)
    {
        // Convert ROS camera matrix to OpenCV format
        camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_info.K.data())).clone();

        // Convert distortion coefficients
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double *>(camera_info.D.data())).clone();
    }

    std::vector<std::vector<double>> ArucoPoseProcessor::processImage(const cv::Mat &input_image)
    {
                std::vector<std::vector<double>> markerPose;
        // Check if camera calibration is loaded
        if (camera_matrix_.empty() || dist_coeffs_.empty())
        {
            ROS_WARN("Camera calibration not set!");
            return markerPose;
        }

        // Detect ArUco markers
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::Mat working_image = input_image.clone();

        cv::aruco::detectMarkers(working_image, aruco_dictionary_, corners, ids, aruco_parameters_, rejected);

        // If markers are detected
        if (!ids.empty())
        {
            // Create marker object points
            std::vector<cv::Point3f> obj_points = createBoardObjectPoints(marker_size_);

            // Process each detected marker
            for (size_t i = 0; i < ids.size(); i++)
            {
                cv::Vec3d rvec, tvec;

                // Estimate pose
                cv::solvePnP(obj_points, corners[i], camera_matrix_, dist_coeffs_,
                             rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

                // Draw axes
                working_image = drawAxis(working_image, corners[i], rvec, tvec,
                                         camera_matrix_, dist_coeffs_);

                // Optional: Add text annotations, print pose, etc.
                ROS_INFO("Marker %d: Position (x,y,z) = (%.2f, %.2f, %.2f)",
                         ids[i], tvec[0], tvec[1], tvec[2]);

                std::vector<double> poseVec = {rvec[0],rvec[1],rvec[2],tvec[0],tvec[1],tvec[2],(double)ids[i]};
                markerPose.push_back(poseVec);
                
            }
        }
        this->debuggingImage = working_image;
        return markerPose;
    }

    cv::Mat ArucoPoseProcessor::drawAxis(cv::Mat &img, const std::vector<cv::Point2f> &corners,
                                         const cv::Vec3d &rvec, const cv::Vec3d &tvec,
                                         const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                                         float length)
    {
        // Define the 3D points for the axes
        std::vector<cv::Point3f> axis_points = {
            {0, 0, 0},
            {length, 0, 0},
            {0, length, 0},
            {0, 0, length}};

        // Project 3D points to image plane
        std::vector<cv::Point2f> imgpts;
        cv::projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs, imgpts);

        // Draw the axes
        cv::line(img, imgpts[0], imgpts[1], cv::Scalar(0, 0, 255), 3); // X-axis (Red)
        cv::line(img, imgpts[0], imgpts[2], cv::Scalar(0, 255, 0), 3); // Y-axis (Green)
        cv::line(img, imgpts[0], imgpts[3], cv::Scalar(255, 0, 0), 3); // Z-axis (Blue)

        return img;
    }

    std::vector<cv::Point3f> ArucoPoseProcessor::createBoardObjectPoints(float marker_size)
    {
        std::vector<cv::Point3f> marker_points = {
            {-marker_size / 2, marker_size / 2, 0},
            {marker_size / 2, marker_size / 2, 0},
            {marker_size / 2, -marker_size / 2, 0},
            {-marker_size / 2, -marker_size / 2, 0}};
        return marker_points;
    }

} // namespace aruco_pose_estimator