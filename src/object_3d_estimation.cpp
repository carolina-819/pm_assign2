#include "object_3d_estimation.h"



void PointCloudToDepthMap(pcl::PointCloud<pcl::PointXYZ>::Ptr)
{
    depth_map = cv::Mat(cam_info.height, cam_info.width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

    for (int i=0; i<pc->points.size();i++)
    {
      if (pc->points[i].z == pc->points[i].z && pc->points[i].z >= 0) // NaN values have the odd property that comparisons involving them are always false
      {
//          ROS_WARN_STREAM("x=" << pc->points[i].x << " y=" << pc->points[i].y << " z=" << pc->points[i].z);
//          ROS_WARN_STREAM("fx=" << left_camera.fx << " fy=" << left_camera.fy);
          float z = pc->points[i].z;
          float u = (pc->points[i].x*left_camera.fx) / z;
          float v = (pc->points[i].y*left_camera.fy) / z;

          int pixel_pos_x = (int)(u + left_camera.cx);
          int pixel_pos_y = (int)(v + left_camera.cy);
//          ROS_WARN_STREAM("u=" << u << " v=" << v << " z=" << z);

          if (pixel_pos_x > (cam_info.width-1) || pixel_pos_y > (cam_info.height-1)) continue;

//          ROS_WARN_STREAM("pos_x=" << pixel_pos_x << " pos_y=" << pixel_pos_y << " z=" << z << "\n");

          depth_map.at<float>(pixel_pos_y,pixel_pos_x) = z;
      }
    }

    depth_map.convertTo(depth_map, CV_8UC1);

//    cv::Mat smaller_img; // copy to vizualize in smaller window, similiar to YOLO window
//    cv::resize(depth_map, smaller_img, cv::Size(600, 500));
//    cv::imshow("PCL image", smaller_img);
////    cv::imshow("Depth Map", depth_map);
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
//    cv::Mat img_bb = left_image.clone();

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
    double middle = (max + min)/ 2;
//    ROS_WARN_STREAM("Middle=" << middle);

    double mid_middle = (middle + min) / 2; // To filter very small BBs


    // Get Bounding Boxes
    for (int i = 0; i < n_cars; i++)
    {
        x = car_BBs.bounding_boxes[i].xmin;
        y = car_BBs.bounding_boxes[i].ymin;
        w = car_BBs.bounding_boxes[i].xmax - x;
        h = car_BBs.bounding_boxes[i].ymax - y;

        double BB_size = w*h;
//        ROS_WARN_STREAM("Size=" << BB_size);

        // Filter Bounding Boxes
        if(BB_size >= mid_middle && h < w*1.5)
        {
            double car_prob = car_BBs.bounding_boxes[i].probability;

            if(car_prob >= 0.35)
            {
                // Accept Only Bounding Boxes With Correct Dimensions
                if(w > 0 && w < cam_info.width && h > 0 && h < cam_info.height)
                {
                    // Saves BBs
                    car_ROIs.push_back(cv::Rect(x, y, w, h));
//                    ROS_WARN_STREAM("x="<<car_ROIs[i].x<<" y="<<car_ROIs[i].y<<" w="<<car_ROIs[i].width<<" h="<<car_ROIs[i].height);

//                      ROS_WARN_STREAM("Probability=" << car_prob*100);
//                      cv::rectangle(img_bb, car_ROIs[i], cv::Scalar(0, 255, 0), 1, 8, 0);
                }
            }
        }
    }

//    // Show chosen BBs
//    cv::Mat mask = cv::Mat::zeros(left_image.size(), left_image.type());
//    cv::Mat segmented = cv::Mat::zeros(left_image.size(), left_image.type());
//    for(int i=0; i<car_ROIs.size(); i++)
//    {
//        cv::rectangle(mask, car_ROIs[i], cv::Scalar(255, 255, 255),-1, 8, 0);
//        left_image.copyTo(segmented, mask);
//    }
//    cv::imshow("segmented", segmented);
//    cv::waitKey(1);

    return car_ROIs;
}


std::vector<cv::Rect> enlargeBBs(std::vector<cv::Rect> car_ROIs)
{
    int n_cars = car_ROIs.size();

    std::vector<cv::Rect> car_ROIs_large;
    car_ROIs_large.reserve(n_cars);

    for(int i=0; i<n_cars; i++)
    {
        int x = car_ROIs[i].x;
        int y = car_ROIs[i].y;
        int w = car_ROIs[i].width;
        int h = car_ROIs[i].height;

        int x_new, y_new, w_new, h_new;

        // Left, Up. Down -> 25% bigger
        // Right -> 33% bigger
        if(x-x/4 >= 0)
        {
            x_new = x- x/4;
            w_new = w + x/4;

            if(x_new + w_new+w/3 < cam_info.width) w_new += w/3;
            else w_new = cam_info.width - x_new;
        }
        else
        {
            x_new = x;
            w_new = w;
        }

        if(y-y/4 >= 0)
        {
            y_new = y - y/4;
            h_new = h + y/4;

            if(y_new + h_new+h/4 < cam_info.height) h_new += h/4;
            else h_new = cam_info.height - y_new;
        }
        else
        {
            y_new = y;
            h_new = h;
        }

        // Save Enlarged BBs
        car_ROIs_large.push_back(cv::Rect(x_new, y_new, w_new, h_new));
//        ROS_WARN_STREAM("x="<<car_ROIs_large[i].x<<" y="<<car_ROIs_large[i].y<<" w="<<car_ROIs_large[i].width<<" h="<<car_ROIs_large[i].height<<"\n");
    }

    return car_ROIs_large;
}


cv::Mat applyKMeans(cv::Mat img)
{
    const cv::Mat& source = img.clone();

    const unsigned int singleLineSize = source.rows * source.cols;
    const unsigned int K = 3;
    cv::Mat data = source.reshape(1, singleLineSize);
    data.convertTo(data, CV_32F);
    std::vector<int> labels;
    cv::Mat1f colors;

    cv::kmeans(data, K, labels, cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 10, 1.), 2, cv::KMEANS_PP_CENTERS, colors);

    for (unsigned int i = 0 ; i < singleLineSize ; i++ )
    {
        if(labels[i] != 2) data.at<float>(i) = 0;
    }

    cv::Mat outputImage = data.reshape(1, source.rows);
//    outputImage.convertTo(outputImage, CV_8U);

//    cv::imshow("K_Means", outputImage);
//    cv::waitKey(1);

    return outputImage;
}


void highlight_depth_map(cv::Mat dp, cv::Rect BB)
{
    cv::Mat depth_bb = dp.clone();
    depth_bb.convertTo(depth_bb,CV_8UC3); // to be able to draw rgb rectangle
    cvtColor(depth_bb, depth_bb, cv::COLOR_GRAY2BGR);

    cv::Mat mask = cv::Mat::zeros(depth_bb.size(), depth_bb.type());
    cv::Mat segmented = cv::Mat::zeros(depth_bb.size(), depth_bb.type());
    cv::rectangle(mask, BB, cv::Scalar(255, 255, 255),-1, 8, 0);
    depth_bb.copyTo(segmented, mask);

    cv::Mat color_dm;
    applyColorMap(segmented, color_dm, cv::COLORMAP_HOT);

    cv::rectangle(color_dm, BB, cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::imshow("Colored Depth Map", color_dm);
    cv::waitKey(1);
}


int getClosestCar(std::vector<cv::Rect> bbs)
{
    cv::Mat dm = depth_map.clone();

    double nz_avg = 0, min_avg = 1000;
    int idx = 0;

    if(bbs.size() > 0)
    {
        for(int i = 0; i < bbs.size(); i++)
        {
            cv::Mat cropped_dp = dm(bbs[i]);

            double s = cv::sum(cropped_dp)[0];
            int n_nz = cv::countNonZero(cropped_dp);

            nz_avg = s / n_nz;
//            ROS_WARN_STREAM("non zero avg = "<<nz_avg);

            if(nz_avg < min_avg)
            {
                min_avg = nz_avg;
                idx = i;
            }
        }

//        ROS_WARN_STREAM("min_avg="<<min_avg);

//        // Show chosen BBs
//        cv::Mat mask = cv::Mat::zeros(left_image.size(), left_image.type());
//        cv::Mat segmented = cv::Mat::zeros(left_image.size(), left_image.type());
//        cv::rectangle(mask, bbs[idx], cv::Scalar(255, 255, 255),-1, 8, 0);
//        left_image.copyTo(segmented, mask);
//        cv::imshow("After mean based filtering", segmented);
//        cv::waitKey(1);

        // Segment Depth Map
        cv::Mat mask_dm = cv::Mat::zeros(depth_map.size(), depth_map.type());
        cv::Mat segmented_dm = cv::Mat::zeros(depth_map.size(), depth_map.type());
        cv::rectangle(mask_dm, bbs[idx], cv::Scalar(255),-1, 8, 0);
        depth_map.copyTo(segmented_dm, mask_dm);
//        cv::Mat color_dm;
//        applyColorMap(segmented_dm, color_dm, cv::COLORMAP_HOT);
//        cv::imshow("Segment Depth Map", color_dm);
//        cv::waitKey(1);

        // Filter Outliars Too Far
        cv::Mat filtered_dm;
        double upper_t = 3*min_avg;
        cv::threshold(segmented_dm, filtered_dm, upper_t, 255, cv::THRESH_TOZERO_INV); // Binarize image // 0: Binary, 1: Binary Inverted, 2: Truncate, 3: To Zero, 4: To Zero Inverted
//        cv::Mat color_dm2;
//        applyColorMap(filtered_dm, color_dm2, cv::COLORMAP_HOT);
//        cv::imshow("Filtered Depth Map - Up", color_dm2);
//        cv::waitKey(1);

        // Filter Outliars Too Close
        double lower_t = min_avg/10;
        cv::threshold(filtered_dm, filtered_dm, lower_t, 255, cv::THRESH_TOZERO); // Binarize image // 0: Binary, 1: Binary Inverted, 2: Truncate, 3: To Zero, 4: To Zero Inverted
        cv::Mat color_dm3;
        applyColorMap(filtered_dm, color_dm3, cv::COLORMAP_HOT);
        cv::imshow("Filtered Depth Map - Up/Low", color_dm3);
        cv::waitKey(1);

        filtered_depth_map = filtered_dm.clone();

        ROS_WARN_STREAM("avg="<<min_avg<<" low="<<lower_t<<" upper="<<upper_t<<"\n");
    }

    return idx;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthMapToPointcloud(cv::Rect roi, cv::Mat dm)
{
    cv::Mat depth = dm.clone();

    cv::Mat image = left_image.clone();
    float z, y, x;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    depth.convertTo(depth, CV_32F);

    if (!depth.data)
    {
        std::cerr << "No depth data!!!" << std::endl;
        exit(EXIT_FAILURE);
    }

    /*if(image.rows != depth.rows || image.cols != depth.cols){
        std::cerr << "Different sizes" << std::endl;
        exit(EXIT_FAILURE);
    }*/

    pointcloud->header.frame_id = "vision_frame";
    pointcloud->width = roi.width; //Dimensions must be initialized to use 2-D indexing
    pointcloud->height = roi.height;

    for (int v = roi.y; v < (roi.y + roi.height); v ++)
    {
        for (int u = roi.x; u < (roi.x + roi.width); u ++)
        {
            if(v )
            {
                //calculates only for points inside roi
                float Z = depth.at<float>(v, u);

                pcl::PointXYZRGB p;
                p.z = Z;
                p.x = ((u - left_camera.cx) * Z / left_camera.fx);
                p.y = ((v - left_camera.cy) * Z / left_camera.fy);
                p.r = image.at<cv::Vec3b>(v, u)[2];
                p.g = image.at<cv::Vec3b>(v, u)[1];
                p.b = image.at<cv::Vec3b>(v, u)[0];


                pointcloud->points.push_back(p);
            }
        }
    }

    return pointcloud;
}


pcl::PointXYZ calculateCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointXYZ aux;

    size_t siz = pointcloud->size();

    for(size_t i = 0; i < siz; i++)
    {
        aux.x = pointcloud->points[i].x;
        aux.y = pointcloud->points[i].y;
        aux.z = pointcloud->points[i].z;
        centroid.add(aux);
    }

    pcl::PointXYZ c;
    centroid.get(c);

    //converter para sensor_msgs
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pointcloud.get(),cloud_msg );

    return c;
}


void getClosestAndPublish(cv::Rect BB, cv::Rect BB_large, cv::Mat depth_map_arg)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = depthMapToPointcloud(BB_large, depth_map_arg);
    pcl::PointXYZ c = calculateCentroid(pointcloud);

    //send message with red bounding box and centroid position
    pm_assign2::bounding redbb;
    redbb.header.stamp = ros::Time::now();
    redbb.xp = c.x;
    redbb.yp = c.y;
    redbb.zp = c.z;
    redbb.x = BB.x;
    redbb.y = BB.y;
    redbb.width = BB.width;
    redbb.height = BB.height;
//    ROS_WARN_STREAM("mais pequeno " << index);

    //converter para sensor_msgs
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pointcloud.get(), cloud_msg);

    pub_cloud_depth.publish(cloud_msg);

    pub_red_bb.publish(redbb);
}


void cbBoundingBoxes(const darknet_ros_msgs::BoundingBoxesConstPtr& msg_BBs)
{
    darknet_ros_msgs::BoundingBoxes car_BBs;

    // Filter Car Bounding Boxes
    for (int i = 0; i < msg_BBs->bounding_boxes.size(); i++)
    {
        if(msg_BBs->bounding_boxes[i].Class == "car")
        {
            car_BBs.bounding_boxes.push_back(msg_BBs->bounding_boxes[i]);
        }
    }

    // Filter Irrelevant Car Bounding Boxes
    std::vector<cv::Rect> car_ROIs = FilterBoundingBoxesRGB(car_BBs);

    // Enlarge BBs for Depth Map Processing
    std::vector<cv::Rect> car_ROIs_large = enlargeBBs(car_ROIs);

    // Get Closest Car From Depth Map
    int idx = getClosestCar(car_ROIs_large);

    // Publish depth map -> pcl RGB
    cv::Mat dm = filtered_depth_map.clone();
    getClosestAndPublish(car_ROIs[idx], car_ROIs_large[idx], dm);
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

    message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/stereo/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_sub, pc_sub);
    sync.registerCallback(boost::bind(&cbNewImage, _1, _2));

    get_left_camera_info();

    ros::Subscriber bb_sub = nh.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);

    pub_cloud_XYZ = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
    pub_cloud_depth = nh.advertise<sensor_msgs::PointCloud2> ("points_depth", 1);
    pub_red_bb = nh.advertise<pm_assign2::bounding> ("closest_car", 1);


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
