## Small Object Discovery
-------------------------

### Introduction
------------------------
Small Object Discovery module is an implementation of the algorithm developed to find an optimal plan for a mobile robot to strategically visit a selected few among all the objects scattered on floor, based on guess from far, recognize when near strategy as described in this [paper](http://robotics.iiit.ac.in/uploads/Main/Publications/Siva_etal_ICVGIP_14.pdf). This module is complete upto discovering objects lying on floor using Markov Random Fields and visiting each object lying in the arena after recognition. This module is being developed at the Robotics Research Centre, IIIT Hyderabad, Hyderabad, India.

### Prerequisites
------------------------
* Turtlebot 1 (Create base)
* Microsoft Kinect
* ROS (Fuerte preferred)
* OpenCV 2.x.x

### Usage
---------------------------

1. Create a ROS package using [roscreate](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) with the following dependencies:
	```
	std_msgs
	roscpp
	tf
	opencv2
	cv_bridge
	image_transport
	message_filters
	geometry_msgs
	nav_msgs
	sensor_msgs
	actionlib_msgs
	move_base_msgs
	actionlib
	pcl_ros
	```

2. Clone this repository locally and copy the content to the package you just created. Add the rosbuild executable path for src/SmallObjectDisc.cpp to your CMakeLists.txt

3. [Build](http://wiki.ros.org/ROS/Tutorials/BuildingPackages) the package

4. Bringup your turtlebot (See [this](http://wiki.ros.org/Robots/TurtleBot) to see how to setup your turtlebot)
	```
	$ roslaunch turtlebot_bringup minimal.launch
	```
	Launch the GMapping module
	```
	$ roslaunch turtlebot_navigation gmapping_demo.launch
	```

5. Run it!
	```
	$ rosrun `name of your package` SmallObjectDisc
	```

### Contributors
* [Prithvijit Chattopadhyay](https://github.com/prithvijit)
* [Ayush Tomar](https://github.com/ayushtomar)
* [Sudhanshu Mittal](https://in.linkedin.com/pub/sudhanshu-mittal/57/786/aa2)
* [Siva Karthik M](https://in.linkedin.com/in/msivak) 

### References
* Sudhanshu et al. Small Object Discovery and Recognition using Actively Guided Robot. In ICPR 2014 [[link]](http://robotics.iiit.ac.in/uploads/Main/Publications/Sudhanshu_etal_ICPR_14.pdf)
* M Siva Karthik et al. Guess from Far, Recognize when Near: Searching the Floor for Small Objects. In ICVGIP 2014 [[link]](http://robotics.iiit.ac.in/uploads/Main/Publications/Siva_etal_ICVGIP_14.pdf)

