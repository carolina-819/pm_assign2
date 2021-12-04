#include "object_3d_estimation.h"

void cbNewPCl (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){

}
void cbNewImage(const sensor_msgs::ImageConstPtr& msg)
{
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
    std::String frame;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/stereo/left/image_rect_color", 1, cbNewImage);
    ros::Subscriber cloud_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> ("velodyne_points", 1, cbNewPCL);

    // se calhar faz sentido fazer sincronizaçao de topicos aqui para garantir que o depth map e calculado sempre para 
    nh.getParam("/object_3d_estimation/frame_id", frame);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
