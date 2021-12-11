#include "object_visualization.h"


void cbNewImage(const sensor_msgs::ImageConstPtr& img_msg)
{
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image.clone();    
}

void cbBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& msg_BBs)
{
    for (int i = 0; i < msg_BBs->bounding_boxes.size(); i++)
    {
        int x = msg_BBs->bounding_boxes[i].xmin;
        int y = msg_BBs->bounding_boxes[i].ymin;
        int w = msg_BBs->bounding_boxes[i].xmax - x;
        int h = msg_BBs->bounding_boxes[i].ymax - y;

        cv::rectangle(left_image, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 1, 8, 0);
    }
}

void cbClosest(const pm_assign2::boundingConstPtr& msg)
{
    cv::Rect bb = cv::Rect(msg->x, msg->y, msg->width, msg->height);
    cv::rectangle(left_image, bb, cv::Scalar(0, 0, 255), 1, 8, 0);
    cv::putText(left_image, "X:" + std::to_string(msg->xp) + " Y:" + std::to_string(msg->yp) + " Z:" +std::to_string(msg->zp), cv::Point(msg->x, msg->y), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,0,255),1, 2,false);
    cv::imshow("Left image", left_image);
    cv::waitKey(1);
}


int main(int argc, char** argv) {

    ros::init(argc, argv, "object_visualization");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    ros::Subscriber left_image_sub = nh.subscribe("/stereo/left/image_rect_color", 1, cbNewImage);
    ros::Subscriber bb_sub = nh.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);
    ros::Subscriber msg_sub = nh.subscribe("closest_car", 1, cbClosest);


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
