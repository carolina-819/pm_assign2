#include "object_3d_estimation.h"

void cbNewImage(const sensor_msgs::ImageConstPtr& msg, const darknet_ros_msgs::BoundingBoxesConstPtr& msgObjects)
{
    ROS_WARN_STREAM("Entered Callback!");

    // Convert ros msg to opencv image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image;
    cv::imshow("Left image", left_image);
    cv::waitKey(1);

//    // Get Bounding Box
//    int n = msgObjects->bounding_boxes.size();
//    int x, y, w, h;
//    std::vector<cv::Rect> objects;
//    objects.reserve(n);
//    for (int i = 0; i < n; i++)
//    {
//        x = msgObjects->bounding_boxes[i].xmin;
//        y = msgObjects->bounding_boxes[i].ymin;
//        w = msgObjects->bounding_boxes[i].xmax - x;
//        h = msgObjects->bounding_boxes[i].ymax - y;
//        objects.push_back(cv::Rect(x, y, w, h));
//    }
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "object_3d_estimation");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/stereo/left/image_rect_color", 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> object_sub(nh, "/objects/left/bounding_boxes", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_sub, object_sub);
    sync.registerCallback(boost::bind(&cbNewImage, _1, _2));


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
