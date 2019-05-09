cmake_minimum_required(VERSION 2.8.3)
project(barcode)

set(Tesseract_DIR /home/balaji/catkin_ws/src/barcode/cmake/Modules)
set(Leptonica_DIR /home/balaji/catkin_ws/src/barcode/cmake/Modules)
set(CMAKE_MODULE_PATH ${barcode_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge 
  image_geometry
  roscpp
  rospy
  std_msgs
  geometry_msgs
  image_transport
  message_generation
)
add_message_files(
   FILES
   qrdata.msg
)
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${Tesseract_INCLUDE_DIRS} ${LEPTONICA_INCLUDE_DIRS})
find_package(OpenCV REQUIRED)
find_package(Tesseract REQUIRED)
find_package (ZBar0 REQUIRED)
catkin_package(
  LIBRARIES barcode
  CATKIN_DEPENDS message_runtime nodelet cv_bridge roscpp std_msgs geometry_msgs 
)
add_executable(barcode_node src/main.cpp)
add_executable(feed src/Imgpub.cpp)
add_executable(tess_node src/textprep.cpp)
add_executable(undistort_node src/undistort.cpp)

target_link_libraries(tess_node tesseract ${catkin_LIBRARIES} ${OpenCV_LIBS} ${TESSERACT_LIBRARIES} ${LEPTONICA_LIBRARIES})
add_dependencies(tess_node barcode_generate_messages_cpp)

target_link_libraries(undistort_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(undistort_node barcode_generate_messages_cpp)

target_link_libraries(barcode_node zbar ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(barcode_node barcode_generate_messages_cpp)

target_link_libraries(feed ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(feed barcode_generate_messages_cpp)




