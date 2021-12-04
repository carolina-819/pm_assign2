#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
 #include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

ros::Publisher pub_cloud_XYZ, pub_cloud_XYZRGB;
PointCloudXYZ::Ptr msg_cloud_xyz;
PointCloudXYZRGB::Ptr msg_cloud_xyz_rgb;

cv::Ptr<cv::Feature2D> feature_detector;
cv::Ptr<cv::DescriptorMatcher> descriptor_matcher;
std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
cv::Mat descriptors_left, descriptors_right;

typedef struct {
    double fx, fy;
    double cx, cy;
} camera_parameters;

camera_parameters left_camera, right_camera;
