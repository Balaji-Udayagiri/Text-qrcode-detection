#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace std;
using namespace cv;


cv::Mat input_img, imgpro;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image msg;
int flag=0;
void imageCallback(const sensor_msgs::ImageConstPtr& img_msg)
{
  try
  {
    input_img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;   
 
    //float cameraMatrix[3][3]={423.776607, 0, 316.324559, 0, 423.936134, 269.224917, 0, 0, 1.0};                                       //SJ4000
	float cameraMatrix[3][3]={537.292878, 0.000000, 427.331854, 0.000000, 527.000348, 240.226888, 0.000000, 0.000000, 1.000000};        //Bebop-2
	//float distCoeffs[5]={-0.386858, 0.123377,-0.014007, -0.003243, 0};																//SJ4000
	float distCoeffs[5]={0.004974, -0.000130, -0.001212, 0.002192, 0.000000};															//Bebop-2
	
	Mat cm= Mat(3,3,CV_32FC1,cameraMatrix);
	Mat dist= Mat(1,5,CV_32FC1,distCoeffs);
	
	undistort(input_img,imgpro,cm,dist,cm);
	//string name = "/home/debjoy/pic_folder/pic" + to_string(flag) + ".jpg";
	//imwrite(name, imgpro);
	flag++;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "Image_Preprocessing");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/bebop/image_raw", 1, imageCallback);
	image_transport::Publisher pub = it.advertise("/usb_cam/processed", 1);

	int counter = 0;
	ros::Rate loop_rate(500);
    while (nh.ok()) {
		counter++;
    	ros::spinOnce(); 
	
		std_msgs::Header header;
        header.seq = counter; 
        header.stamp = ros::Time::now(); 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imgpro);
        img_bridge.toImageMsg(msg); 
        pub.publish(msg);
        loop_rate.sleep();
    }
	return 0;
}
