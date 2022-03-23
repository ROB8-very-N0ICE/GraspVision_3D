#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>


void rgbCallback(const sensor_msgs::ImageConstPtr& msg){
    try{

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC3);
        cv::Mat rgb_img = cv_ptr->image;
        cv::Mat rgb_img_show = cv_ptr->image;

        //calculate the distance
        int w = 320;
        int h = 240;
        int offset = 30;
        cv::Point pt1(w, h);
        cv::Point pt2(w + offset, h + offset);
        cv::rectangle(rgb_img_show, pt1, pt2, 65535, 10);

        cv::imshow("RGB", rgb_img_show);
        cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void pointCloudCallback(const sensor_msgs::ImageConstPtr& msg){
    try{
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        cv::Mat depth = cv_ptr->image;
        cv::Mat depth_show = cv_ptr->image;
        // rgb      cols= 640   rows = 480
        // depth    cols= 848   rows = 480


        for(int i= 0 ; i < depth.rows-1; i++){
            for (int j = 0; j < depth.cols-1; ++j) {
                depth_show.at<int>(i, j) = depth.at<int>(i, j) / pow(10, 5) * 65535;
            }
        }


        int w = depth.cols;
        int h = depth.rows;
        int offset = 30;
        int point = depth.at<int>((w+offset)/2, (h+offset)/2);
        ROS_INFO("distance: %f", point);    //TODO: must calculate unities

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    //image_transport::Subscriber subI = it.subscribe("/camera/color/image_raw", 1, rgbCallback);
    image_transport::Subscriber subD = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, pointCloudCallback);

    ros::spin();
    cv::destroyWindow("view");
}