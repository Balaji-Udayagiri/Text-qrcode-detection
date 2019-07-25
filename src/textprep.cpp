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
#include <fstream>
#include <bits/stdc++.h> 
using namespace std;
using namespace cv;
using namespace zbar;

cv::Mat im;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg, dummy_msg;
string outText;
int qr_data_is_incoming=0;
ofstream MyExcelFile;
string qr_data;
string comma = ",";
int conf;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if(qr_data_is_incoming=0)
	return;
  try
  {
    im = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
	
	auto numOfConfigs = 1;
	auto **configs    = new char* [numOfConfigs];
	configs[0] = (char *) "bazaar.config";

    tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
     
    // Initialize tesseract to use English (eng) and the LSTM OCR engine. 
    ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY, configs, numOfConfigs, nullptr, nullptr, false);
 	//ocr->Init(NULL, "eng", tesseract::OEM_LSTM_ONLY);

 	ocr->SetVariable("tessedit_char_whitelist","qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM0123456789");
    
	// Set Page segmentation mode to PSM_AUTO (3)
   // ocr->SetPageSegMode(tesseract::PSM_SINGLE_WORD);
	ocr->SetPageSegMode(tesseract::PSM_AUTO);
 
    // Open input image using OpenCV
    
    
	ocr->SetImage(im.data, im.cols, im.rows, 3, im.step);
 
    // Run Tesseract OCR on image

	ocr->Recognize(0);
  	tesseract::ResultIterator* ri = ocr->GetIterator();
  	tesseract::PageIteratorLevel level = tesseract::RIL_WORD;
  	if (ri != 0) {
    	do {
    	  const char* word = ri->GetUTF8Text(level);
    	  float conf = ri->Confidence(level);
    	  
    	  int x1, y1, x2, y2;
    	  cout<<"\n"<<word<<"\n";
    	  ri->BoundingBox(level, &x1, &y1, &x2, &y2);
    	  printf("word: '%s';  \tconf: %.2f; BoundingBox: %d,%d,%d,%d; width: %d; height: %d\n",word, conf, x1, y1, x2, y2, abs(x2-x1), abs(y2-y1));
    	  rectangle( im, Point( x1, y1 ), Point( x2, y2 ), Scalar( 0, 0, 255 ), +1, 4 );
    	  if(conf>80 && word!=" ")
    	  	outText = string(word);
    	  	//outText = string(ocr->GetUTF8Text());
    	  delete[] word;
    	} while (ri->Next(level));
  	}
  	
	ROS_INFO("text detected: [%s]", outText.c_str());
	if(strlen(outText.c_str())>0)
		{
		outText.erase(std::remove(outText.begin(),outText.end(),' '),outText.end());
		string cc = qr_data+","+outText+","+" ";
		
		cc.erase(std::remove(cc.begin(),cc.end(),'\n'),cc.end());
		
		if(strlen(cc.c_str() )>( strlen(qr_data.c_str() )+4)  )
		MyExcelFile <<cc<< endl;
		}
	ocr->End();	
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void QRdataCallback(const barcode::qrdata::ConstPtr& msg1)
{
  qr_data_is_incoming = 0;
  ROS_INFO("Qr data detected: [%s]", msg1->data.data.c_str());
  if(strlen(msg1->data.data.c_str())>0)
  {
	qr_data_is_incoming=1;
	ROS_INFO("in func");
 	qr_data=msg1->data.data.c_str();
  }
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "textprep");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	
	MyExcelFile.open("/home/ark/qdata/inventory.csv");
	MyExcelFile << "QR-code data, AlphaNumeric Code, Shelf Code" << endl;
	
	ros::Subscriber data_sub = nh.subscribe("/code/data",1,QRdataCallback);
	image_transport::Subscriber sub = it.subscribe("/code/image", 1, imageCallback);

	ros::spin();
	MyExcelFile.close();
	return 0;
}
		
	
	
