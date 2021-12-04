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
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf/transform_listener.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

cv::Mat left_image;

