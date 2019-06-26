# rpi_opencv_cam
ROS nodes for using a raspberry pi camera with openCV

## Installation
Requires [raspicam_node by Ubiquity Robotics](https://github.com/UbiquityRobotics/raspicam_node)  
Add rpi_opencv_cam to your catkin_ws/src folder, then run catkin_make from your catkin_ws directory

## Usage
  1. With roscore running, run raspicam_node  
    V2.x camera: roslaunch raspicam_node camerav2_1280x960.launch  
    V1.x camera: roslaunch raspicam_node camerav1_1280x720.launch  
  2. Run the desired node in rpi_opencv_cam by using:  
    rosrun rpi_opencv_cam node_name.py  
    * note that rpi_opencv_color_detect_node requires command line arguments to specify which colors to look for  
  3. View results using rqt_image_view  
  
## Miscellaneous Info
  * Master branch corresponds to the version made for running on raspberry pi, using the [Ubiquity Robotics Xenial image](https://downloads.ubiquityrobotics.com/pi.html)
  * Raspberry pi currently runs opencv 3.3.1-dev, and desktop runs opencv 4.1.0
  * Both branches use python 2.7
  * Desktop runs ROS melodic, raspberry pi runs ROS kinetic
  
## License
[MIT](https://choosealicense.com/licenses/mit/)
