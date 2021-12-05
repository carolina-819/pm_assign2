#include "object_3d_estimation.h"
sensor_msgs::PointCloud2 cloud_framed;
ros::Publisher pub_cloud_XYZ;
tf::TransformListener *listener;


void cbNewImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
//    ROS_WARN_STREAM("Entered Callback!");

    // Convert ros msg to opencv image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image;
    
    //process point cloud

    
   
    sensor_msgs::PointCloud2 temp_out;
    pcl_ros::transformPointCloud(frame_img, *pc_msg, temp_out, *listener); 
  std::cout << "wat" << std::endl;
    pub_cloud_XYZ.publish(temp_out);
   
//    cv::imshow("Left image", left_image);
//    cv::waitKey(1);
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

    cv::imshow("Left image w/ BBs", img_bb);
    cv::waitKey(1);

    ROS_WARN_STREAM("Number of boxes = " << objects.size());
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


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
