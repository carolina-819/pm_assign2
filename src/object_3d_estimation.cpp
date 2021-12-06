#include "object_3d_estimation.h"



void PointCloudToDepthMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)
{
    cv::Mat cv_image = cv::Mat(cam_info.height, cam_info.width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

    for (int i=0; i<pc->points.size();i++)
    {
      if (pc->points[i].z == pc->points[i].z) // NaN values have the odd property that comparisons involving them are always false
      {
          ROS_WARN_STREAM("x=" << pc->points[i].x << " y=" << pc->points[i].y << " z=" << pc->points[i].z);
//          ROS_WARN_STREAM("fx=" << left_camera.fx << " fy=" << left_camera.fy);
          float z = pc->points[i].z;
          float u = (pc->points[i].x*left_camera.fx) / z;
          float v = (pc->points[i].y*left_camera.fy) / z;

          int pixel_pos_x = (int)(u + left_camera.cx);
          int pixel_pos_y = (int)(v + left_camera.cy);
//          ROS_WARN_STREAM("u=" << u << " v=" << v << " z=" << z);

          if (pixel_pos_x > (cam_info.width-1)) pixel_pos_x = cam_info.width -1;
          if (pixel_pos_y > (cam_info.height-1)) pixel_pos_y = cam_info.height-1;

          z = z * 1000;
          ROS_WARN_STREAM("pos_x=" << pixel_pos_x << " pos_y=" << pixel_pos_y << " z=" << z);

          cv_image.at<float>(pixel_pos_y,pixel_pos_x) = z;

      }
    }

    cv_image.convertTo(cv_image,CV_16UC1);
    cv::imshow("PCL image", cv_image);
    cv::waitKey(1);
}


void cbNewImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
//    ROS_WARN_STREAM("Entered Callback!");

    // Convert ros msg to opencv image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image;
//    cv::imshow("Left image", left_image);
//    cv::waitKey(1);

    
    //process point cloud   
    sensor_msgs::PointCloud2 temp_out;
    pcl_ros::transformPointCloud(frame_img, *pc_msg, temp_out, *listener);
    pub_cloud_XYZ.publish(temp_out);


    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(temp_out, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc);

    ROS_WARN_STREAM("PCL -> Width = " << pc->width << " Height = " << pc->height);


    PointCloudToDepthMap(pc);
}


void cbBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& msgObjects)
{
    cv::Mat img_bb = left_image.clone();

    // Get Bounding Box
    int n = msgObjects->bounding_boxes.size();
    int x, y, w, h;
    std::vector<cv::Rect> objects;
    objects.reserve(n);
    for (int i = 0; i < n; i++)
    {
        x = msgObjects->bounding_boxes[i].xmin;
        y = msgObjects->bounding_boxes[i].ymin;
        w = msgObjects->bounding_boxes[i].xmax - x;
        h = msgObjects->bounding_boxes[i].ymax - y;

        objects.push_back(cv::Rect(x, y, w, h));

        cv::rectangle(img_bb, objects[i], cv::Scalar(0, 0, 255), 1, 8, 0);
    }

//    cv::imshow("Left image w/ BBs", img_bb);
//    cv::waitKey(1);

//    ROS_WARN_STREAM("Number of boxes = " << objects.size());
}


void get_left_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;

    do {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/stereo/left/camera_info", ros::Duration(5));

        if(sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            left_camera.fx = cam_info.K[0];
            left_camera.cx = cam_info.K[2];
            left_camera.fy = cam_info.K[4];
            left_camera.cy = cam_info.K[5];

            ROS_WARN_STREAM("Left camera:");
            ROS_WARN_STREAM("cx = " << left_camera.cx << " cy = " << left_camera.cy);
            ROS_WARN_STREAM("fx = " << left_camera.fx << " fy = " << left_camera.fy);

            ROS_WARN_STREAM("Width = " << cam_info.width << " Height = " << cam_info.height);
        }
        else
        {
//            ROS_ERROR("Couldn't get left camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while(sharedCameraInfo == NULL);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "object_3d_estimation");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    listener = new tf::TransformListener();
    
    nh.getParam("/object_3d_estimation/frame_id_img", frame_img);
    nh.getParam("/object_3d_estimation/frame_id_pcl", frame_pcl);

    pub_cloud_XYZ =  nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/stereo/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_sub, pc_sub);
    sync.registerCallback(boost::bind(&cbNewImage, _1, _2));

    ros::Subscriber bb_sub = nh.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);

    get_left_camera_info();


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
