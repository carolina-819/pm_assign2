#include "object_visualization.h"



void cbNewImage(const sensor_msgs::ImageConstPtr& img_msg){
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image.clone();
   /* cv::Rect bb = cv::Rect(msg->x, msg->y, msg->width, msg->height);
    cv::rectangle(left_image, bb, cv::Scalar(255, 0, 0), 1, 8, 0);
    cv::putText(left_image, std::to_string(msg->xp) + ", " + std::to_string(msg->yp) + ", " +std::to_string(msg->zp), cv::Point(msg->x, msg->y), cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::imshow("Left image", left_image);
    cv::waitKey(1);*/
    
}

void cbClosest(const pm_assign2::boundingConstPtr& msg){
    cv::Rect bb = cv::Rect(msg->x, msg->y, msg->width, msg->height);
    cv::rectangle(left_image, bb, cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::putText(left_image, std::to_string(msg->xp) + ", " + std::to_string(msg->yp) + ", " +std::to_string(msg->zp), cv::Point(msg->x, msg->y), cv::FONT_HERSHEY_DUPLEX,1,cv::Scalar(0,255,0),2,false);
    cv::imshow("Left image", left_image);
    cv::waitKey(1);
    
}
int main(int argc, char** argv) {

    ros::init(argc, argv, "object_visualization");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

  
   ros::Subscriber left_image_sub = nh.subscribe("/stereo/left/image_rect_color", 1, cbNewImage);
    ros::Subscriber bb_sub = nh.subscribe("closest_car", 1, cbClosest);
    
    
   //sincronizaçao da camara com a bounding a funcionar mal
   /*
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/stereo/left/image_rect_color", 1);
    message_filters::Subscriber<pm_assign2::bounding> bb_sub(nh, "closest_car", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, pm_assign2::bounding> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_sub, bb_sub);
    sync.registerCallback(boost::bind(&cbNewImage, _1, _2));*/
    
    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
