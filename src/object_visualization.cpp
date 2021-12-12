#include "object_visualization.h"
int x, y, w, h;

/*void PointCloudToDepthMap(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
    depth_map_cut = cv::Mat(cam_info.height, cam_info.width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));

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

*/

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
        if(msg_BBs->bounding_boxes[i].Class == "car"){
            x = msg_BBs->bounding_boxes[i].xmin;
            y = msg_BBs->bounding_boxes[i].ymin;
            w = msg_BBs->bounding_boxes[i].xmax - x;
            h = msg_BBs->bounding_boxes[i].ymax - y;

            cv::rectangle(left_image, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 1, 8, 0);
        }
        
    }
}


void cbClosest(const pm_assign2::boundingConstPtr& msg)
{
    cv::Rect bb = cv::Rect(msg->x, msg->y, msg->width, msg->height);
    
    // Transform centroid to "base_link"
    geometry_msgs::PointStamped centroid;
    centroid.header.frame_id = "vision_frame";
    centroid.header.stamp = ros::Time();
    centroid.point.x = msg->xp;
    centroid.point.y = msg->yp;
    centroid.point.z = msg->zp;
    float w = msg->car_w;
    float h = msg->car_h;


    geometry_msgs::PointStamped transformed_centroid;
    listener->transformPoint("base_link", centroid, transformed_centroid);

    float cx=transformed_centroid.point.x, cy=transformed_centroid.point.y, cz=transformed_centroid.point.z;

    // Warning Sign
    if(cx < 10 && cy < 5 && cy > -5)
    {
        cv::rectangle(left_image, bb, cv::Scalar(0, 0, 255), 1, 8, 0);
        cv::putText(left_image, "X:" + std::to_string(cx), cv::Point(msg->x, msg->y-40), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,0,255),1, 2,false);
        cv::putText(left_image, "Y:" + std::to_string(cy), cv::Point(msg->x, msg->y-20), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,0,255),1, 2,false);
        cv::putText(left_image, "Z:" +std::to_string(cz), cv::Point(msg->x, msg->y), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,0,255),1, 2,false);
        cv::putText(left_image, "width: " +std::to_string(w) + "height: " +std::to_string(h), cv::Point(msg->x, msg->y-60), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,0,255),1, 2,false);
    
    }

    else
    {
//        cv::rectangle(left_image, bb, cv::Scalar(0, 255, 0), 1, 8, 0);
        cv::putText(left_image, "X:" + std::to_string(cx), cv::Point(msg->x, msg->y-40), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,255,0),1, 2,false);
        cv::putText(left_image, "Y:" + std::to_string(cy), cv::Point(msg->x, msg->y-20), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,255,0),1, 2,false);
        cv::putText(left_image, "Z:" +std::to_string(cz), cv::Point(msg->x, msg->y), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,255,0),1, 2,false);
        cv::putText(left_image, "width: " +std::to_string(w) + "height: " +std::to_string(h), cv::Point(msg->x, msg->y-60), cv::FONT_HERSHEY_DUPLEX,0.7,cv::Scalar(0,255,0),1, 2,false);
    
    }
          
    cv::imshow("Left image", left_image);
    cv::waitKey(1);
}

void cbDepth(const sensor_msgs::PointCloud2ConstPtr& pc_msg)
{
  /*  pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl_conversions::toPCL(pc_msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, pc);
    PointCloudToDepthMap(pc);
    cv::Mat aux = depth_map_cut.clone();
    aux.convertTo(aux,CV_8UC3); // to be able to draw rgb rectangle
    cvtColor(aux, aux, cv::COLOR_GRAY2BGR);

    cv::Mat heat;
    applyColorMap(aux, heat, cv::COLORMAP_HOT);

     cv::imshow("caliente", heat);
    cv::waitKey(1);*/
   // cv::Mat prev = left_image[x:(x+w), y:(y+h), :];
   // cv::bitwise_or(prev , heat, left_image[x:(x+w), y:(y+h), :]);

}


int main(int argc, char** argv) {

    ros::init(argc, argv, "object_visualization");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    listener = new tf::TransformListener();

    ros::Subscriber left_image_sub = nh.subscribe("/stereo/left/image_rect_color", 1, cbNewImage);
    ros::Subscriber bb_sub = nh.subscribe("/objects/left/bounding_boxes", 1, cbBoundingBoxes);

    ros::Subscriber msg_sub = nh.subscribe("closest_car", 1, cbClosest);

   // ros::Subscriber pcl_heat_sub = nh.subscribe("points_depth", 1, cbDepth);


    while(ros::ok())
    {
        ros::spinOnce();
    }

    cv::destroyAllWindows();
    return 0;
}
