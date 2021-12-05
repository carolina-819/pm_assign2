#include "object_3d_estimation.h"
std::string frame;
pcl::PointCloud<pcl::PointXYZRGB> cloud_framed;
void cbNewPCL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
   // tf::TransformListener listener;
    //tf::StampedTransform transform;
   // listener.lookupTransform("velodyne", frame, ros::Time(0), transform);
    //transform pointcloud to target frame given from param
   // pcl_ros::transformPointCloud(*msg, cloud_framed, transform);	
   
    //pub_cloud_XYZRGB.publish(cloud_framed);
    std::cout << "wiwowi: " << std::endl;

}
void cbNewImage(const sensor_msgs::ImageConstPtr& msg)
{
    std::cout << "wiiiwowo" << std::endl;
    // Convert ros msg to opencv image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat left_image = cv_data->image;
    cv::imshow("Left image", left_image);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_3d_estimation");
    ros::NodeHandle nh;
 //   nh.getParam("/object_3d_estimation/frame_id", frame);
    
 //   pub_cloud_XYZRGB =  nh.advertise<PointCloudXYZRGB> ("points2", 1);
 //   image_transport::ImageTransport it(nh);
  //  image_transport::Subscriber image_sub = it.subscribe("/stereo/left/image_rect_color", 1, cbNewImage);
  //  ros::Subscriber cloud_sub = nh.subscribe("/velodyne_points", 1, cbNewPCL);

    // se calhar faz sentido fazer sincronizaçao de topicos aqui para garantir que o depth map e calculado sempre para 
    
    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
