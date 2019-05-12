# Text-qrcode-detection
Package name: barcode

Packages used:

OpenCV
Zbar
Tesseract-ocr

Ros Nodes:

1.Node:feed - executable:Imgpub.cpp 

  Publishes camera channel 1 images in ROS-topic /usb_cam/image_raw

2.Node:undistort_node - executable:undistort.cpp

  Subscribes to images from /usb_cam/image_raw and publishes    undistorted images in ROS-topic /usb_cam/processed (reference camera SJ-4000)

3.Node:barcode_node - executable:main.cpp

  Subscribes to images from /usb_cam/processed and extracts QR-code information (coordinates of bounding rectangle and qr-code information). Publishes bounding rectangle containing image in ROS-topic /code/image and information in /code/data. 

4.Node:tess_node - executable:textprep.cpp

  Subscribes to ROS-topics /code/image and /code/data and prints any alphanumeric text and/or qr-code information (for now). To further develop.

msg type used:
qrdata.msg
sensor_msgs::Image


To Run:
roslaunch barcode qrtext.launch 


References:
 for msg of type sensor_msgs::Image ----
 http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages

for make file of tesseract ---- 
https://stackoverflow.com/questions/38128260/cmake-and-tesseract-how-to-link-using-cmake?fbclid=IwAR3-wI5U-YVLXltUM0lDAtnVXMRqkxGh2l94o8BPXgBjYTASn0UBBZxiKk4

and

https://github.com/opencv/opencv/issues/11486






