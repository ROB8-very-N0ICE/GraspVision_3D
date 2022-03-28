#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <yolact_ros_msgs/Detections.h>


cv::Mat rgb_img;
int width = 640;    // hardcodding... to make it faster
int height = 480;

cv::Mat m1 = cv::Mat::zeros(height, width, CV_8UC1); // test masks

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
    //yolact_ros_msgs::Detection_<std::allocator<void> > foo = detec.detections[20];
    //std::cout << name << std::endl;
    //ROS_INFO("Msg: " << &name);
    //if (rgb_img != NULL) {
    for(int i=0; i<5; i++){ // only 5 detections per message for some reason
      std::string class_name = detec.detections[i].class_name;
      float score = detec.detections[i].score; // char64_t its float 64, it may cahnge in a diff machine must controll it later
      std::vector<int_least32_t> box = {
        detec.detections[i].box.x1,
        detec.detections[i].box.y1,
        detec.detections[i].box.x2,
        detec.detections[i].box.y2
      };


      std::vector<int> size = {
        detec.detections[i].mask.width,
        detec.detections[i].mask.height
      };

      //np.zeros([height, width, 3], dtype=np.uint8)
      //std::cout << (size[0] <= width) << " " << (size[1] <= height) << std::endl;

      if (class_name == "person") {
        m1 = cv::Mat::zeros(height, width, CV_8UC1);
        for (size_t i = box[0]; i < box[2]-1; i++) {
          for (size_t j = box[1]; j < box[3]-1; j++) {
            m1.data[(j*width) + i] = 250;

            //rgb_img.data[rgb_img.step[0]*i + rgb_img.step[1]* j + 0] = 0;
            //rgb_img.data[rgb_img.step[0]*i + rgb_img.step[1]* j + 1] = 0;
            //rgb_img.data[rgb_img.step[0]*i + rgb_img.step[1]* j + 2] = 0;
          };
        };
        //m1.data[200] = 254;
        //std::cout << width << " " << height << '\n';
        cv::imshow("mask", m1);
        //cv::waitKey(30);
      };

      //std::cout << box[0] << " " << box[1] << " " << box[2] << " " << box[3] << " " << size[0] <<" "<< size[1] << " "<<  class_name <<std::endl;
      //if (rgb_img.data[10]) {
        //std::cout << rgb_img.data[10] << '\n';
        //cv::imshow("mask", rgb_img);
        //cv::waitKey(30);
      //}
    };
    //TODO: overlap masks to point cloud
    //TODO: remove further points
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera_node");
    //ros::Rate r(5); // 5 hz
    ros::NodeHandle nh;
    cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subD = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, pointCloudCallback);
    ros::Subscriber subDet = nh.subscribe("/yolact_ros/detections", 1, detectionsCallback);
    ros::spin();
    cv::destroyWindow("view");
}
