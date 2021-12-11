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

    cv::Mat smaller_img; // copy to vizualize in smaller window, similiar to YOLO window
    cv::resize(depth_map, smaller_img, cv::Size(600, 500));
    cv::imshow("PCL image", smaller_img);
//    cv::imshow("Depth Map", depth_map);
    cv::waitKey(1);
}

pcl::PointXYZ depthMapToPointcloud(cv::Rect roi, bool publish){
    
    cv::Mat depth = depth_map.clone();
    cv::Mat image = left_image.clone();
    float z, y, x;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
     depth.convertTo(depth, CV_32F);

      if (!depth.data) {
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
            if(v ){ //calculates only for points inside roi
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
    
    //pcl::PointXYZ res(0,0,0);
    pcl::CentroidPoint<pcl::PointXYZ> centroid;
    pcl::PointXYZ aux;
    size_t siz = pointcloud->size();
    for(size_t i = 0; i < siz; i++){
        aux.x = pointcloud->points[i].x;
        aux.y = pointcloud->points[i].y;
        aux.z = pointcloud->points[i].z;
        centroid.add(aux);
    }
    pcl::PointXYZ c1;
    centroid.get(c1);
    
    //converter para sensor_msgs
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pointcloud.get(),cloud_msg );

    if(publish) pub_cloud_depth.publish(cloud_msg);
    return c1;
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

std::vector<int> enlargeBBs(int x, int y, int w, int h)
{
    int x_new, y_new, w_new, h_new;

    if(x-x/3 >= 0)
    {
        x_new = x- x/3;
        w_new = w + x/3;

        if(x_new + w_new+w/3 < cam_info.width) w_new += w/3;
        else w_new = cam_info.width - x_new;
    }
    else
    {
        x_new = x;
        w_new = w;
    }

    if(y-y/3 >= 0)
    {
        y_new = y - y/3;
        h_new = h + y/3;

        if(y_new + h_new+h/3 < cam_info.height) h_new += h/3;
        else h_new = cam_info.height - y_new;
    }
    else
    {
        y_new = y;
        h_new = h;
    }

    std::vector<int> new_points;
    new_points.reserve(4);

    new_points.push_back(x_new);
    new_points.push_back(y_new);
    new_points.push_back(w_new);
    new_points.push_back(h_new);

    return new_points;
}

std::vector<cv::Rect> FilterBoundingBoxesRGB(darknet_ros_msgs::BoundingBoxes car_BBs)
{
//    cv::Mat img_bb = left_image.clone();

    int n_cars = car_BBs.bounding_boxes.size();

    std::vector<cv::Rect> car_ROIs;
    pcl::PointXYZ c, c_final;
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
//                    ROS_WARN_STREAM("x="<<x<<" y="<<y<<" w="<<w<<" h="<<h);

                    // Expand BBs (respecting limits)
                    std::vector<int> new_points = enlargeBBs(x, y, w, h);
                    x = new_points[0];
                    y = new_points[1];
                    w = new_points[2];
                    h = new_points[3];

                    // Save BBs
                    car_ROIs.push_back(cv::Rect(x, y, w, h));
//                    ROS_WARN_STREAM("x="<<car_ROIs[i].x<<" y="<<car_ROIs[i].y<<" w="<<car_ROIs[i].width<<" h="<<car_ROIs[i].height<<"\n");

//                      ROS_WARN_STREAM("Probability=" << car_prob*100);
//                      cv::rectangle(img_bb, car_ROIs[i], cv::Scalar(0, 255, 0), 1, 8, 0);
                }
            }
        }
    }
    int index = 0;
    if(car_ROIs.size() != 0)
    {
        //crop image and depth_map, send to calculate point cloud, get centroid
        
        /*
        cv::Mat mask = cv::Mat::zeros(left_image.size(), left_image.type());
        cv::Mat segmented = cv::Mat::zeros(left_image.size(), left_image.type());
*/      float min = 100;
       
        
        for(int i=0; i < car_ROIs.size(); i++)
        {
            cv::Mat og_img = left_image.clone(), og_cm = depth_map.clone();
            cv::Mat img_cr = og_img(car_ROIs[i]);
            cv::Mat dm_cr = og_cm(car_ROIs[i]);
            c = depthMapToPointcloud(car_ROIs[i], false);
            if(c.z < min){ min = c.z; index = i; c_final = c;}
            //cv::rectangle(mask, car_ROIs[i], cv::Scalar(255,255,255),-1, 8, 0);
        }

        //left_image.copyTo(segmented, mask);

      //  cv::imshow("Segmented", segmented);
      //  cv::waitKey(1);
    }

//        cv::imshow("Left image w/ BBs", img_bb);
//        cv::waitKey(1);
//send message with red bounding box and centroid position
    pm_assign2::bounding redbb;
    redbb.header.stamp = ros::Time::now();
    redbb.xp = c_final.x;
    redbb.yp = c_final.y;
    redbb.zp = c_final.z;
    redbb.x = car_ROIs[index].x;
    redbb.y = car_ROIs[index].y;
    redbb.width = car_ROIs[index].width;
    redbb.height = car_ROIs[index].height;
    ROS_WARN_STREAM("mais pequeno " << index);

    cv::Mat og_i = left_image.clone(), og_d= depth_map.clone();
    cv::Mat icr = og_i(car_ROIs[index]);
    cv::Mat dcr = og_d(car_ROIs[index]);
    depthMapToPointcloud(car_ROIs[index], true);
    pub_red_bb.publish(redbb);

    return car_ROIs;
}


void getClosestCar(std::vector<cv::Rect> bbs)
{
    cv::Mat dm = depth_map.clone();

    double avg = 0, min_avg = 1000;
    int idx;

    for(int i = 0; i < bbs.size(); i++)
    {
        cv::Mat cropped_dp = dm(bbs[i]);

        avg = cv::mean(cropped_dp)[0];
        ROS_WARN_STREAM("mean="<<avg);

        if(avg < min_avg)
        {
            min_avg = avg;
            idx = i;
        }
    }

    ROS_WARN_STREAM("min_avg="<<min_avg);

    // Visualize Closest BB's Point Cloud With Color
    cv::Mat depth_bb = depth_map.clone();
    depth_bb.convertTo(depth_bb,CV_8UC3); // to be able to draw rgb rectangle
    cvtColor(depth_bb, depth_bb, cv::COLOR_GRAY2BGR);

    cv::Mat mask = cv::Mat::zeros(depth_bb.size(), depth_bb.type());
    cv::Mat segmented = cv::Mat::zeros(depth_bb.size(), depth_bb.type());
    cv::rectangle(mask, bbs[idx], cv::Scalar(255, 255, 255),-1, 8, 0);
    depth_bb.copyTo(segmented, mask);

    cv::Mat color_dm;
    applyColorMap(segmented, color_dm, cv::COLORMAP_HOT);

    cv::rectangle(color_dm, bbs[idx], cv::Scalar(0, 255, 0), 1, 8, 0);

    cv::imshow("Colored Depth Map", color_dm);

    cv::waitKey(1);
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

    int n = car_BBs.bounding_boxes.size();

    // Filter Irrelevant Car Bounding Boxes
    std::vector<cv::Rect> car_ROIs;
    car_ROIs = FilterBoundingBoxesRGB(car_BBs);

    // Get Closest Car From Depth Map
    getClosestCar(car_ROIs);
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

    pub_cloud_XYZ = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
    pub_cloud_depth = nh.advertise<sensor_msgs::PointCloud2> ("points_depth", 1); 
    pub_red_bb = nh.advertise<pm_assign2::bounding> ("closest_car", 1);
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
