#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "mindvision_gige_ros/Save_Image.h"

#include <string>

// sensor_msgs::Image img1, img2;
cv_bridge::CvImagePtr cv_img1_ptr, cv_img2_ptr;
bool cam1_up, cam2_up;

void cam1_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        cv_img1_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cam1_up = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void cam2_cb(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_img2_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cam2_up = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

bool save_srv(mindvision_gige_ros::Save_Image::Request& req,
                mindvision_gige_ros::Save_Image::Response& res)
{
    static int imgCount;
    cam1_up = false; cam2_up = false;

    ROS_INFO("Waiting for image ...");
    while(!(cam1_up && cam2_up) && ros::ok()) { std::cout << "."; }
    std::cout << std::endl;

    ROS_INFO("%d Img Saved !" , imgCount);
    std::string imgpath = "/home/pibot/imgs/mindvision_save/";
    std::string imgLName = imgpath +std::to_string(imgCount)+"L.png";
    std::string imgRName = imgpath +std::to_string(imgCount)+"R.png";

    imwrite(imgLName, cv_img1_ptr->image);
    imwrite(imgRName, cv_img2_ptr->image);

    imgCount++;

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_save_node");
    ros::NodeHandle nh;


    ros::Subscriber cam1_sub = nh.subscribe("mindvision1/image", 1, &cam1_cb);
    ros::Subscriber cam2_sub = nh.subscribe("mindvision2/image", 1, &cam2_cb);

    ros::ServiceServer save_server = nh.advertiseService("image_saver", save_srv);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    while(ros::ok())
    {
        // cv_img1_ptr->image
        // cv::Mat img1mat(cvSize(img1.width, img1.height), CV_8UC3);
        // cv::Mat img2mat(cvSize(img2.width, img2.height), CV_8UC3);

        // cv_bridge::CvImage()
    }
}