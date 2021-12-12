#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include "pcl_ros/point_cloud.h"
#include <pcl_ros/transforms.h>
#include <boost/shared_ptr.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>
#include "pm_assign2/bounding.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <pcl/common/centroid.h>
#include <numeric>
#include <string>
#include <functional>

cv::Mat left_image;
cv::Mat depth_map;
cv::Mat filtered_depth_map;

std::string frame_img, frame_pcl;

typedef struct {
    double fx, fy;
    double cx, cy;
} camera_parameters;

camera_parameters left_camera;

sensor_msgs::CameraInfo cam_info;

pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);


sensor_msgs::PointCloud2 cloud_framed, msg_cloud_xyz;

ros::Publisher pub_cloud_XYZ, pub_cloud_depth, pub_red_bb, pub_cloud_val, pub_image;

tf::TransformListener *listener;

#define PI 3.14159265

ros::Publisher pub_Pose;
