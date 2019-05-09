QR and Barcode Reader 

Implements zbar for ROS. Subscribes to '/usb_cam/image_raw' and publishes detected results under '/code'. 

```/code/image``` displays final image after processing

```/code/qr``` contains the qr code data if one is encountered.

```/code/bar``` contains any other barcode data if one in encountered.

```/code/data``` contains qr code data and bounding box coordinates 

Requiremnets :

OpenCV

Zbar

usb_cam



How to run:

Through Laptop webcam feed
(in 3 different terminals, run.. )

rosrun barcode barcode_node
rosrun barcode feed
rostopic echo /code/..   ### Whatever Topic 


For images published in topic 'xxx'

rosrun barcode barcode_node (Replace subscribed topic "/usb_cam/image_raw" with xxx)
rostopic echo /code/..   ### Whatever Topic 


