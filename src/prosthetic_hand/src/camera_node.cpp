#include <ros/ros.h>
#include <image_transport/image_transport.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <yolact_ros_msgs/Detections.h>

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
        ROS_INFO("distance: %d", point);    //TODO: must calculate unities

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
    for(int i=0; i<5; i++){ // only 5 detections per message for some reason
      std::string class_name = detec.detections[i].class_name;

      float score = detec.detections[i].score; // its float 64, it may cahnge in a diff machine must controll it later

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


      //cv::Mat img = cv::Mat::ones(height, width, CV_8UC1)*250;
      //cv::Mat img = cv::Mat::create(mask, height, width, CV_8UC1);

      std::cout << box[0] << " " << box[1] << " " << box[2] << " " << box[3] << " " << size[0] <<" "<< size[1] << " "<<  class_name <<std::endl;


      //cv::Mat img(width, height, CV_8UC1);
      //cv::cvtColor(mask, img, CV_8UC1);
      //img = img.reshape(width, height);
      //cv::imshow("mask", img);


      //for(int i=0; i<width; ++i){
        //for(int j=0; j<height; ++j){
          //img.at<uint8_t>(i, j) = 255; //mask[i*j];
        //}
      //}

      //Mat img(width, height, CV_8UC1, Scalar(0,0, 100));
      //std::cout << height << "\t" << width << " " << sizeof(mask) << std::endl;

      //cv::Mat img(width, height, cv::CV_64FC1);
      //for(int i=0; i<width; ++i){
        //for(int j=0; j<height; ++j){
          //img.at<auto>(i, j) = mask.at(i).at(j);
        //}
      //}



      //cv::Mat binary = cv::imdecode(detec.detections[i].mask.mask, cv::IMREAD_ANYCOLOR);

      //cv::waitKey(30);

      //cv::Mat matAngles(angles.size(), angles.at(0).size(), sensor_msgs::image_encodings::TYPE_8UC1);
//for(int i=0; i<matAngles.rows; ++i)
     //for(int j=0; j<matAngles.cols; ++j)
          //matAngles.at<double>(i, j) = angles.at(i).at(j);





      //cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(mask, sensor_msgs::image_encodings::TYPE_8UC1);
      //cv_bridge::CvImagePtr cv_ptr = cv::imdecode (mask, sensor_msgs::image_encodings::TYPE_8UC1)
      //cv::Mat binary = cv_ptr->image;
      //cv::Mat rgb_img_show = cv_ptr->image;

      //if (name=="person") {
      //cv::imshow("mask", binary);
      //cv::waitKey(30);
      //}

      //std::cout << i << "\t"<< name << certainty << std::endl;

    }

    //TODO: iterate through detections
    //TODO: get mask-class pair
    //TODO: overlap masks to point cloud
    //TODO: remove further points

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    cv::startWindowThread();
    //image_transport::ImageTransport it(nh);
    //image_transport::Subscriber subD = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, pointCloudCallback);
    ros::Subscriber subDet = nh.subscribe("/yolact_ros/detections", 1, detectionsCallback);

    ros::spin();
    cv::destroyWindow("view");
}
