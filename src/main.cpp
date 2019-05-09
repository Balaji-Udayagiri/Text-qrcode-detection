#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "zbar.h"
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "barcode/qrdata.h"
using namespace std;
using namespace cv;
using namespace zbar;

cv::Mat frame;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "image_subscriber");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/usb_cam/processed", 1, imageCallback);
	image_transport::Publisher pub = it.advertise("/code/image", 1);
	//ros::Publisher qr_pub = nh.advertise<std_msgs::String>("/code/qr", 10);
	//ros::Publisher pos_pub = nh.advertise<geometry_msgs::Polygon>("/code/bound", 4);
        //ros::Publisher bar_pub = nh.advertise<std_msgs::String>("/code/bar", 10);
	ros::Publisher data_pub = nh.advertise<barcode::qrdata>("/code/data",10);
	
    // Create a zbar reader
    ImageScanner scanner;
    
    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    ros::Rate loop_rate(5);
    while (nh.ok()) {
        // Capture an OpenCV frame
        ros::spinOnce();
        if(!frame.cols)
        	continue;
        cv::Mat frame_grayscale;
        // Convert to grayscale
        cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

        // Obtain image data
        int width = frame_grayscale.cols;
        int height = frame_grayscale.rows;
        uchar *raw = (uchar *)(frame_grayscale.data);

        // Wrap image data
        Image image(width, height, "Y800", raw, width * height);
	geometry_msgs::Polygon P;
        // Scan the image for barcodes
        //int n = scanner.scan(image);
        scanner.scan(image);

        // Extract results
        int counter = 0;
        std_msgs::String qr_msg,bar_msg;
	barcode::qrdata msg;
        for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {

            if(symbol->get_type_name() == "QR-Code"){
            	qr_msg.data = symbol->get_data();
            	//qr_pub.publish(qr_msg);
            }
            else{
            	bar_msg.data = symbol->get_data();
            	//bar_pub.publish(bar_msg);
            }
            if (symbol->get_location_size() == 4) {
                //rectangle(frame, Rect(symbol->get_location_x(i), symbol->get_location_y(i), 10, 10), Scalar(0, 255, 0));
                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
            }
            counter++;
	
		for(int i=0; i<4; i++)
		{
			geometry_msgs::Point32 t1;
			t1.x = symbol->get_location_x(i);
			t1.y = symbol->get_location_y(i);
			t1.z = 0;
			P.points.push_back(t1);				
		}
	msg.Poly = P;
	msg.data = qr_msg;
	//pos_pub.publish(P);
	data_pub.publish(msg);
        }
	
        std_msgs::Header header;
        header.seq = counter; 
        header.stamp = ros::Time::now(); 
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, frame);
        img_bridge.toImageMsg(img_msg); 
        //sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(img_msg);
        image.set_data(NULL, 0);
	imshow("win", frame);
	waitKey(1);
    }

    return 0;
}
