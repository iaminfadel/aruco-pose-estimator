#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "aruco_pose_estimator/aruco_pose_processor.h"

class ArucoPoseNode
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Subscriber camera_info_sub_;
    ros::Publisher landmarks_pub_;
    aruco_pose_estimator::ArucoPoseProcessor pose_processor_;
    camera_info_manager::CameraInfoManager camera_info_manager_;
    cv_bridge::CvImage debugImage;
    visualization_msgs::MarkerArray landmarks;

public:
    ArucoPoseNode() : it_(nh_), camera_info_manager_(nh_)
    {
        // Get parameters from launch file or default values
        std::string image_topic, camera_info_topic;
        nh_.param<std::string>("image_topic", image_topic, "/camera/image_raw");
        nh_.param<std::string>("camera_info_topic", camera_info_topic, "/camera/camera_info");

        // Subscribe to image and camera info topics
        image_sub_ = it_.subscribe(image_topic, 1, &ArucoPoseNode::imageCallback, this);
        camera_info_sub_ = nh_.subscribe(camera_info_topic, 1, &ArucoPoseNode::cameraInfoCallback, this);
        landmarks_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/landmarks", 10);
        image_pub_ = it_.advertise("/ar_tracker/debug_image", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Process the image
        std::vector<std::vector<double>> markerPose = pose_processor_.processImage(cv_ptr->image);
        cv_ptr->image = pose_processor_.debuggingImage;
        image_pub_.publish(cv_ptr->toImageMsg());

        visualization_msgs::MarkerArray markerArray;
        

        for (int i = 0; i < markerPose.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "camera_optical_frame";
            marker.header.stamp = ros::Time::now();

            marker.ns = "MARKER";
            marker.id = (int)markerPose[i][6];

            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            marker.pose.position.x = markerPose[i][3];
            marker.pose.position.y = markerPose[i][4];
            marker.pose.position.z = markerPose[i][5];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.14;
            marker.scale.y = 0.14;
            marker.scale.z = 0.01;

            marker.color.r = 4.0/(markerPose[i][6]);
            marker.color.g = 4.0/(5-markerPose[i][6]);
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.lifetime = ros::Duration(0.1);

            markerArray.markers.push_back(marker);
        }
        landmarks_pub_.publish(markerArray);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        // Set camera calibration information
        pose_processor_.setCameraInfo(*msg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_pose_estimator");
    ArucoPoseNode node;
    ros::spin();
    return 0;
}