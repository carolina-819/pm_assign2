#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
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
#include <sensor_msgs/RegionOfInterest.h>
#include "pm_assign2/bounding.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <message_filters/time_synchronizer.h>


cv::Mat depth_mat_cut;

tf::TransformListener *listener;

int x, y, w, h;

int msg_x, msg_y;

std::vector<cv::Rect> BBs;
cv::Rect bb;

geometry_msgs::PointStamped centroid;
geometry_msgs::PointStamped transformed_centroid;

float width, height;

float cx, cy, cz;
