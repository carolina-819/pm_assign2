#include "object_3d_estimation.h"



void PointCloudToDepthMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)
{
    depth_map = cv::Mat(cam_info.height, cam_info.width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

    for (int i=0; i<pc->points.size();i++)
    {
      if (pc->points[i].z == pc->points[i].z) // NaN values have the odd property that comparisons involving them are always false
      {
//          ROS_WARN_STREAM("x=" << pc->points[i].x << " y=" << pc->points[i].y << " z=" << pc->points[i].z);
//          ROS_WARN_STREAM("fx=" << left_camera.fx << " fy=" << left_camera.fy);
          float z = pc->points[i].z;
          float u = (pc->points[i].x*left_camera.fx) / z;
          float v = (pc->points[i].y*left_camera.fy) / z;

          int pixel_pos_x = (int)(u + left_camera.cx);
          int pixel_pos_y = (int)(v + left_camera.cy);
//          ROS_WARN_STREAM("u=" << u << " v=" << v << " z=" << z);

          if (pixel_pos_x > (cam_info.width-1)) pixel_pos_x = cam_info.width -1;
          if (pixel_pos_y > (cam_info.height-1)) pixel_pos_y = cam_info.height-1;

//          z = z * 1000;
//          ROS_WARN_STREAM("pos_x=" << pixel_pos_x << " pos_y=" << pixel_pos_y << " z=" << z << "\n");

          if(z>=0) depth_map.at<float>(pixel_pos_y,pixel_pos_x) = 255 - z * 255 / 50;

      }
    }

    depth_map.convertTo(depth_map,CV_8UC1);

//    cv::Mat smaller_img; // copy to vizualize in smaller window, similiar to YOLO window
//    cv::resize(depth_map, smaller_img, cv::Size(600, 500));
//    cv::imshow("PCL image", smaller_img);
//    cv::waitKey(1);
}


void cbNewImage(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
//    ROS_WARN_STREAM("Entered Callback!");

    // Convert ros msg to opencv image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    left_image = cv_data->image.clone();
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

//    ROS_WARN_STREAM("PCL -> Width = " << pc->width << " Height = " << pc->height);


    PointCloudToDepthMap(pc);
}


std::vector<cv::Rect> FilterBoundingBoxesRGB(darknet_ros_msgs::BoundingBoxes car_BBs)
{
    cv::Mat img_bb = left_image.clone();

    int n_cars = car_BBs.bounding_boxes.size();

    std::vector<cv::Rect> car_ROIs;
    car_ROIs.reserve(n_cars);

    int x, y, w, h;


    // Get Car Bounding Box Sizes
    std::vector<double> sizes;
    sizes.reserve(n_cars);
    for (int i = 0; i < n_cars; i++)
    {
            x = car_BBs.bounding_boxes[i].xmin;
            y = car_BBs.bounding_boxes[i].ymin;
            w = car_BBs.bounding_boxes[i].xmax - x;
            h = car_BBs.bounding_boxes[i].ymax - y;

            sizes.push_back(w*h);
    }

    // Filter outliars
    double max = *max_element(sizes.begin(), sizes.end());
    double min = *min_element(sizes.begin(), sizes.end());
    double middle = (max+min)/2;
    ROS_WARN_STREAM("Middle=" << middle);


    // Get Bounding Boxes
    std::vector<cv::Rect> objects;

    for (int i = 0; i < n_cars; i++)
    {
        x = car_BBs.bounding_boxes[i].xmin;
        y = car_BBs.bounding_boxes[i].ymin;
        w = car_BBs.bounding_boxes[i].xmax - x;
        h = car_BBs.bounding_boxes[i].ymax - y;

        double BB_size = w*h;
        ROS_WARN_STREAM("Size=" << BB_size);

        // Filter Bounding Boxes
        if(BB_size >= middle && h < w*1.5)
        {
            objects.push_back(cv::Rect(x, y, w, h));

            double car_prob = car_BBs.bounding_boxes[i].probability;

            if(car_prob < 0.35)
            {
//                ROS_WARN_STREAM("Red=" << car_prob*100 << "\n");
//                cv::rectangle(img_bb, objects[i], cv::Scalar(0, 0, 255), 1, 8, 0);
            }
            else
            {
//                ROS_WARN_STREAM("Green=" << car_prob*100);
                cv::rectangle(img_bb, objects[i], cv::Scalar(0, 255, 0), 1, 8, 0);

                // Accept Only Bounding Boxes With Correct Dimensions
                if(objects[i].width > 0 && objects[i].width < cam_info.width && objects[i].height > 0 && objects[i].height < cam_info.height)
                {
                    car_ROIs.push_back(objects[i]);
//                    ROS_WARN_STREAM("x="<<car_ROIs[i].x<<" y="<<car_ROIs[i].y<<" w="<<car_ROIs[i].width<<" h="<<car_ROIs[i].height<<"\n");
                }
            }
        }
    }

    if(car_ROIs.size() != 0)
    {
        cv::Mat mask = cv::Mat::zeros(left_image.size(), left_image.type());
        cv::Mat segmented = cv::Mat::zeros(left_image.size(), left_image.type());

        for(int i=0; i< car_ROIs.size(); i++)
        {
            cv::rectangle(mask, car_ROIs[i], cv::Scalar(255,255,255),-1, 8, 0);

        }

        left_image.copyTo(segmented, mask);

        cv::imshow("Segmented", segmented);
        cv::waitKey(1);
    }

//        cv::imshow("Left image w/ BBs", img_bb);
//        cv::waitKey(1);

    return car_ROIs;
}


void cbBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& msg_BBs)
{
    cv::Mat img_bb = left_image.clone();

    cv::Mat depth_bb = depth_map.clone();
    depth_bb.convertTo(depth_bb,CV_8UC3); // to be able to draw rgb rectangle
    cvtColor(depth_bb, depth_bb, cv::COLOR_GRAY2BGR);

    darknet_ros_msgs::BoundingBoxes car_BBs;

    // Filter Car Bounding Boxes
    for (int i = 0; i < msg_BBs->bounding_boxes.size(); i++)
    {
        if(msg_BBs->bounding_boxes[i].Class == "car")
        {
            car_BBs.bounding_boxes.push_back(msg_BBs->bounding_boxes[i]);
        }
    }

    int n = car_BBs.bounding_boxes.size();

    // Filter Irrelevant Car Bounding Boxes
    std::vector<cv::Rect> car_ROIs;
    car_ROIs = FilterBoundingBoxesRGB(car_BBs);
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
