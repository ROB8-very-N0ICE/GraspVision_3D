#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <yolact_ros_msgs/Detections.h>


cv::Mat rgb_img;
int width = 10;    // hardcodding... to make it faster
int height = 10;
cv::Mat mask_img = cv::Mat::zeros(height, width, CV_8UC1); // test masks
cv::Mat depth = cv::Mat::zeros(height, width, CV_16UC1);

void pointCloudCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        cv::Mat depth = cv_ptr->image;
        height = depth.rows;
        width = depth.cols;
        //ROS_INFO("%d\t%d", wi);
        // cv::Mat depth_show = cv_ptr->image;
        // rgb      cols= 640   rows = 480
        // depth    cols= 848   rows = 480

/*
        for(int i= 0 ; i < depth.rows-1; i++){
            for (int j = 0; j < depth.cols-1; ++j) {
                depth_show.at<int>(i, j) = depth.at<int>(i, j) / pow(10, 5) * 65535;
            }
        }
        */
        //width = depth.cols;
        //height = depth.rows;
        //int offset = 30;
        //int point = depth.at<int>((width+offset)/2, (height+offset)/2);
        //ROS_INFO("width:%d height:%d", width, height);    //TODO: must calculate unities

        /*//calculate the distance
        cv::Point pt1(w, h);
        cv::Point pt2(w + offset, h + offset);
        cv::rectangle(depth_show, pt1, pt2, 65535, 10);
        cv::imshow("depth", depth_show);
        cv::waitKey(30);
        */
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void detectionsCallback(const yolact_ros_msgs::Detections &detec){
    for(int i=0; i<5; i++){ // only 5 detections per message for some reason
      std::string class_name = detec.detections[i].class_name;
      float score = detec.detections[i].score; // char64_t its float 64, it may cahnge in a diff machine must controll it later

      if ((class_name == "person") && (score > .5) && width && height) {  // discar uncertainty below .8
        size_t index;
        size_t byte_ind;
        size_t bit_ind;
        size_t val;
        mask_img = cv::Mat::zeros(height, width, CV_8UC1);

        std::vector<uint32_t> box = {//100, 100, 200, 200};
          detec.detections[i].box.x1,
          detec.detections[i].box.y1,
          detec.detections[i].box.x2,
          detec.detections[i].box.y2
        };


        std::vector<int> size = {
          detec.detections[i].mask.width,
          detec.detections[i].mask.height
        };

        for (int j = box[0]; j < box[2]; j++){
          for (int k = box[1]; k < box[3]; k++){
            index = (k - box[1]) * size[0] + j - box[0];
            byte_ind = index / 8;
            bit_ind = 7 - (index % 8); // bitorder 'big'
            val = detec.detections[i].mask.mask[byte_ind] & (1 << bit_ind);
            mask_img.at<uint8_t>(k, j) = ((bool)val) * 255;
            //clean_depth.at<uint16_t>(k, j) = depth.at<uint16_t>(i, j) * ((bool)val)//operate the depth data in the same loop is faster
            //
            //ROS_INFO("%d, %d", j, k);
            };
          };
        };
      };
    cv::imshow("mask", mask_img);
    //TODO: overlap masks to point cloud
    //TODO: remove further points
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_node");
    //ros::Rate r(5); // 5 hz
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subD = it.subscribe("/usb_cam/image_raw", 1, pointCloudCallback); ///camera/aligned_depth_to_color/image_raw", 1, pointCloudCallback);
    ros::Subscriber subDet = nh.subscribe("/yolact_ros/detections", 1, detectionsCallback);
    ros::spin();
    cv::destroyWindow("view");
}
