#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include <string>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <opencv2/opencv.hpp>
#include "zbar.h"
#include "barcode/qrdata.h"

using namespace std;
using namespace cv;
using namespace zbar;

cv::Mat im;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;
string outText;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    im = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
	
     tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
     
    // Initialize tesseract to use English (eng) and the LSTM OCR engine. 
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);
 	
 	ocr->SetVariable("tessedit_char_whitelist","qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM0123456789");
    // Set Page segmentation mode to PSM_AUTO (3)
    ocr->SetPageSegMode(tesseract::PSM_AUTO);
 
    // Open input image using OpenCV
    
    
	ocr->SetImage(im.data, im.cols, im.rows, 3, im.step);
 
    // Run Tesseract OCR on image
    outText = string(ocr->GetUTF8Text());
 ROS_INFO("Data: [%s]", outText.c_str());
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void QRdataCallback(const barcode::qrdata::ConstPtr& msg1)
{
  ROS_INFO("Data: [%s]", msg1->data.data.c_str());
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "textprep");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/code/image", 1, imageCallback);
	ros::Subscriber data_sub = nh.subscribe("/code/data",1,QRdataCallback);
	
	ros::spin();
	return 0;
}
		
	
	
