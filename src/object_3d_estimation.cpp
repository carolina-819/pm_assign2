#include "object_3d_estimation.h"

void detect_features_and_descriptors(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors) {
    keypoints.clear();
    feature_detector->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
};

void calc_disparity_sparse(const std::vector<cv::Point2f>& feat_l, const std::vector<cv::Point2f>& feat_r, std::vector<float>& disparity) {
    for(auto i=0; i < feat_l.size(); i++) {
        disparity[i] = sqrt((feat_l[i].x - feat_r[i].x) * (feat_l[i].x - feat_r[i].x));
    }
}
void convert_disparity_to_depth(const std::vector<float>& disp, const std::vector<cv::Point2f>& feat_ref, const camera_parameters camera, const float baseline, const float minDepth,
                                const float maxDepth, std::vector<cv::Point3f> &feat_ref_3d, size_t &num_valid_depths) {
    num_valid_depths = 0;
    for(auto i=0; i < feat_ref.size(); i++) {
        if(disp[i] > 0) {
            cv::Point3f pt;
            pt.x = -1000;
            pt.y = -1000;
            pt.z = -1000;
            // Calculate Depth
            pt.z = camera.fx * baseline / (disp[i]+1E-5);
            if(pt.z >= minDepth && (pt.z <= maxDepth)) {
                pt.x = (feat_ref[i].x - camera.cx) * pt.z / (camera.fx + 1E-5);
                pt.y = (feat_ref[i].y - camera.cy) * pt.z / (camera.fy + 1E-5);
                num_valid_depths++;
            }
            feat_ref_3d.push_back(pt);
        }
    }
}

void callback(const sensor_msgs::ImageConstPtr &left_img, const sensor_msgs::ImageConstPtr &right_img) {
    // Convert ROS msg to OpenCV image
    cv_bridge::CvImageConstPtr cv_data;
    cv_data = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat left_cv_img = cv_data->image;
    cv_data = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::BGR8);
    cv::Mat right_cv_img = cv_data->image;
    // Extract all features and descriptors from both images
    detect_features_and_descriptors(left_cv_img, keypoints_left, descriptors_left);
    detect_features_and_descriptors(right_cv_img, keypoints_right, descriptors_right);
    // Match the two set of features and filter potential inconsistencies
    std::vector<cv::DMatch> all_matches;
    std::vector<cv::DMatch> good_matches;
    descriptor_matcher->match(descriptors_left, descriptors_right, all_matches);
    std::vector<cv::Point2f> good_keypoints_left, good_keypoints_right;
    for(auto i=0; i < all_matches.size(); i++) {
        if(all_matches[i].distance < 15) {
            good_matches.push_back(all_matches[i]);
            good_keypoints_left.push_back(keypoints_left[all_matches[i].queryIdx].pt);
            good_keypoints_right.push_back(keypoints_right[all_matches[i].trainIdx].pt);
        }
    }
    // Draw good matches
    cv::Mat img_matches;
    cv::drawMatches(left_cv_img, keypoints_left, right_cv_img, keypoints_right, good_matches, img_matches);
    // Show matches
    cv::resize(img_matches, img_matches, cv::Size(800, 800));
    cv::imshow("Matches", img_matches);
    cv::waitKey(1);

    // Calculate the disparity
    std::vector<float> disparity(keypoints_left.size());
    calc_disparity_sparse(good_keypoints_left, good_keypoints_right, disparity);

    // Convert disparity into depth
    std::vector<cv::Point3f> depth_points;
    size_t num_valid_depth_points;
    convert_disparity_to_depth(disparity, good_keypoints_left, left_camera, 6.1217793733703944e-02, 0.1, 200, depth_points, num_valid_depth_points);

    // Publish PointCloudXYZ
    msg_cloud_xyz.reset(new PointCloudXYZ);
    msg_cloud_xyz->header.frame_id = "left_optical";
    pcl_conversions::toPCL(ros::Time::now(), msg_cloud_xyz->header.stamp);
    msg_cloud_xyz->height = 1;
    msg_cloud_xyz->width = num_valid_depth_points;
    msg_cloud_xyz->points.resize(msg_cloud_xyz->width);
    for(auto i=0, j=0; i < depth_points.size(); i++) {
        if(depth_points[i].x == -1000 || depth_points[i].y == -1000)
            continue;
        msg_cloud_xyz->points[j].x = depth_points[i].x;
        msg_cloud_xyz->points[j].y = depth_points[i].y;
        msg_cloud_xyz->points[j].z = depth_points[i].z;
        j++;
    }
    pub_cloud_XYZ.publish(msg_cloud_xyz);

    // Publish PointCloudXYZRGB
    msg_cloud_xyz_rgb.reset(new PointCloudXYZRGB);
    msg_cloud_xyz_rgb->header.frame_id = "left_optical";
    pcl_conversions::toPCL(ros::Time::now(), msg_cloud_xyz_rgb->header.stamp);
    msg_cloud_xyz_rgb->height = 1;
    msg_cloud_xyz_rgb->width = depth_points.size();
    msg_cloud_xyz_rgb->points.resize(msg_cloud_xyz_rgb->width);
    for(auto i=0, j=0; i < msg_cloud_xyz_rgb->width; i++) {
        if(depth_points[i].x == -1000 || depth_points[i].y == -1000)
            continue;
        msg_cloud_xyz_rgb->points[j].x = depth_points[i].x;
        msg_cloud_xyz_rgb->points[j].y = depth_points[i].y;
        msg_cloud_xyz_rgb->points[j].z = depth_points[i].z;
        auto point_rgb = left_cv_img.ptr<cv::Point3_<uchar>>(good_keypoints_left[i].x, good_keypoints_left[i].y);
        msg_cloud_xyz_rgb->points[j].r = point_rgb->z;
        msg_cloud_xyz_rgb->points[j].g = point_rgb->y;
        msg_cloud_xyz_rgb->points[j].b = point_rgb->x;
        j++;
    }
    pub_cloud_XYZRGB.publish(msg_cloud_xyz_rgb);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "calculate_depth_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    message_filters::Subscriber<sensor_msgs::Image> left_image_sub(nh, "/stereo/left/image_rect_color", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_image_sub(nh, "/stereo/right/image_rect_color", 1);

    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    sensor_msgs::CameraInfo camera_parameters_msg;
    // Get Left Camera Info
    do {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/stereo/left/camera_info", ros::Duration(5));
        if(sharedCameraInfo != NULL){
            camera_parameters_msg = *sharedCameraInfo;
            left_camera.fx = camera_parameters_msg.K[0];
            left_camera.cx = camera_parameters_msg.K[2];
            left_camera.fy = camera_parameters_msg.K[4];
            left_camera.cy = camera_parameters_msg.K[5];
            ROS_INFO("Left camera: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", left_camera.fx, left_camera.fy, left_camera.cx, left_camera.cy);
        }
        else {
            ROS_ERROR("Couldn't get left camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while(sharedCameraInfo == NULL);

    // Get Right Camera Info
    do {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/stereo/right/camera_info", ros::Duration(3));
        if(sharedCameraInfo != NULL){
            camera_parameters_msg = *sharedCameraInfo;
            right_camera.fx = camera_parameters_msg.K[0];
            right_camera.cx = camera_parameters_msg.K[2];
            right_camera.fy = camera_parameters_msg.K[4];
            right_camera.cy = camera_parameters_msg.K[5];
            ROS_INFO("Right camera: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", right_camera.fx, right_camera.fy, right_camera.cx, right_camera.cy);
        }
        else {
            ROS_ERROR("Couldn't get right camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while(sharedCameraInfo == NULL);

    // Create Point Cloud publishers
    pub_cloud_XYZ = nh.advertise<PointCloudXYZ>("/stereo/pointcloud", 1);
    pub_cloud_XYZRGB = nh.advertise<PointCloudXYZRGB>("/stereo/pointcloud_color", 1);

    // Create stereo camera links
    static auto static_tf_broadcaster = new tf2_ros::StaticTransformBroadcaster();
    geometry_msgs::TransformStamped static_tf_stamped;
    static_tf_stamped.header.stamp = ros::Time::now();
    static_tf_stamped.header.frame_id = "left_optical";
    static_tf_stamped.child_frame_id = "right_optical";
    static_tf_stamped.transform.translation.x = 6.1217793733703944e-02;
    static_tf_stamped.transform.translation.y = 0.3485175467944353e-03;
    static_tf_stamped.transform.translation.z = -2.5844163627702960e-03;
    static_tf_stamped.transform.rotation.x = 0.0;
    static_tf_stamped.transform.rotation.y = 0.0;
    static_tf_stamped.transform.rotation.z = 0.0;
    static_tf_stamped.transform.rotation.w = 1.0;
    static_tf_broadcaster->sendTransform(static_tf_stamped);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_image_sub, right_image_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Create feature detector
    feature_detector = cv::ORB::create();
    descriptor_matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);

    while(ros::ok()) {
        ros::spinOnce();
    }
    cv::destroyAllWindows();
    return 0;
}
