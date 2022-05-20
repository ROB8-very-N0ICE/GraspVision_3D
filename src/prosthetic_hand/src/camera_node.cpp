#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <yolact_ros_msgs/Detections.h>
#include <std_msgs/UInt8.h>


cv::Mat rgb_img;
int width = 10;    // hardcodding... to make it faster
int height = 10;
//std_msgs::UInt8 machine_state;
//machine_state.data = (uint8_t) 0;
cv::Mat mask_img = cv::Mat::zeros(height, width, CV_8UC1); // test masks
cv::Mat depth = cv::Mat::zeros(height, width, CV_16UC1);
image_transport::Publisher people_pub;
image_transport::Publisher unknown_object_pub;
//ros::Publisher feedback_pub;

void pointCloudCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        depth = cv_ptr->image;
        if (width == 10 && height == 10) {
            height = depth.rows;
            width = depth.cols;
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

sensor_msgs::ImagePtr handle_detections(const yolact_ros_msgs::Detection &detection){
    size_t index;
    size_t byte_ind;
    size_t bit_ind;
    size_t val;
    mask_img = cv::Mat::zeros(height, width, CV_8UC1);

    std::vector <uint32_t> box = {
            detection.box.x1,
            detection.box.y1,
            detection.box.x2,
            detection.box.y2
    };

    std::vector<int> size = {
            detection.mask.width,
            detection.mask.height
    };

    cv::Mat clean_depth = cv::Mat::zeros(size[1], size[0], CV_16UC1);

    for (int x = box[0]; x < box[2]; x++) {
        for (int y = box[1]; y < box[3]; y++) {
            index = (y - box[1]) * size[0] + x - box[0];
            byte_ind = index / 8;
            bit_ind = 7 - (index % 8);  // bitorder 'big'
            val = detection.mask.mask[byte_ind] & (1 << bit_ind);
            if (val) {
                mask_img.at<uint8_t>(y, x) = ((bool) val) * 255;
                //build the depth "mask"
                clean_depth.at<uint16_t>(y - box[1], x - box[0]) = depth.at<uint16_t>(y, x);
            }
        }
    }
    cv::imshow(detection.class_name, mask_img);
    return cv_bridge::CvImage(std_msgs::Header(), "mono16", clean_depth).toImageMsg();
}

sensor_msgs::ImagePtr handle_unknown_object(const yolact_ros_msgs::Detection &detection){
    //mask_img = cv::Mat::ones(height, width, CV_8UC1) * 255;
    cv::Mat clean_depth = depth;

    float upper_threshold = 400; // in millimeters
    float lower_threshold = 200;

    std::vector <uint32_t> box = {
            detection.box.x1,
            detection.box.y1,
            detection.box.x2,
            detection.box.y2
    };

    std::vector<int> size = {
            detection.mask.width,
            detection.mask.height
    };


    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            uint16_t z = clean_depth.at<uint16_t>(y, x);
            if ((z < lower_threshold) || (upper_threshold < z)){
                clean_depth.at<uint16_t>(y, x) = 0;
                //mask_img.at<uint8_t>(y, x) = 0;
            }else if ((box[0] < x) && (x < box[2]) && (box[1] < y) && (y < box[3])){
                size_t index = (y-box[1]) * size[0] + (x-box[0]);
                size_t byte_ind = index / 8;
                size_t bit_ind = 7 - (index % 8);  // bitorder 'big'
                size_t val = detection.mask.mask[byte_ind] & (1 << bit_ind);
                if (val){  //assuming object in hand
                    clean_depth.at<uint16_t>(y, x) = 0;
                    //mask_img.at<uint8_t>(y, x) = 0;
                    //ROS_INFO("z: %d", depth.at<uint16_t>(y, x));
                }
            }
        }
    }
    //cv::imshow("unknown object", mask_img);
    //feedback_pub.publish(1);  // 1 is the next logical state for the state_machine
    ROS_INFO(depth2point(clean_depth)); //saves the point cloud to a file
    return cv_bridge::CvImage(std_msgs::Header(), "mono16", clean_depth).toImageMsg();
}

void detectionsCallback(const yolact_ros_msgs::Detections &detec) {
    //ROS_INFO("Detection!");
    for (int i = 0; i < 5; i++) {  // only 5 detections per message for some reason
        std::string class_name = detec.detections[i].class_name;
        float score = detec.detections[i].score;  // char64_t its float 64, it may cahnge in a diff machine must controll it later
        if ((class_name == "person") && (score > .5)) {
            //people_pub.publish(handle_detections(detec.detections[i]));
            unknown_object_pub.publish(handle_unknown_object(detec.detections[i])); // this assumes only one person detection
            //ROS_INFO("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
        }
    //TODO: remove further points
        }
    }

void state_update(const std_msgs::UInt8 &state){
  //machine_state = state;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr depth2point(){
  // TODO: import pcl
  ROS_INFO("depth_handler------------------------------------------");
  int div = 1;
  int stride = 4;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pointer( new pcl::PointCloud<pcl::PointXYZ>);
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cloud_pointer->width = width;
  cloud_pointer->height = height;
  cloud_pointer->is_dense = false ;
  cloud_pointer->points.resize (width * height);
  float fx = 1.0;  //_intrinsics(0, 0);
  float fy = 1.0;  //_intrinsics(1, 1);
  float cx = 1.0;  //_intrinsics(0, 2);
  float cy = 1.0;  //_intrinsics(1, 2);
  for (int i = 0; i < height; i += stride){
    for (int j = 0; j < width; j += stride){
        float Z = depth.at<uint16_t>(i, j) / 1;
        if(Z){
        cloud_pointer->points[i].x = (i - cx) * Z / fx / div;
        cloud_pointer->points[i].y = (j - cy) * Z / fy / div;
        cloud_pointer->points[i].z = Z / div;
        }
    }
  }
  //int pcl::FileWriter::write 	save_success(const std::string &"cloud_from_camera.ply", const pcl::PointCloud<PointXYZ> &cloud_pointer)
  pcl::io::savePCDFileASCII ("cloud_from_camera.pcd", &cloud);
  return cloud_pointer;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_node");
    //ros::Rate r(5); // 5 hz
    ros::NodeHandle nh;
    cv::startWindowThread();
    ROS_INFO("Camera node starts!-----------------------------------------");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subD = it.subscribe("camera/aligned_depth_to_color/image_raw", 1, pointCloudCallback);
    //people_pub = it.advertise("people_depth", 1);
    unknown_object_pub = it.advertise("unknown_depth", 1);
    //feedback_pub = nh.advertise<std_msgs::UInt8>("state_feedback", 1);
    //usb_cam/image_raw", 1, pointCloudCallback);
    ros::Subscriber subDet = nh.subscribe("/yolact_ros/detections", 1, detectionsCallback);
    //ros::Subscriber state_sub = nh.subscribe("/state_feedforward", 1, state_update);
    ros::spin();
    cv::destroyWindow("view");
}
