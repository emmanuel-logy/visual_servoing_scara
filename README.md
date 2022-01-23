# Visual Servoing - SCARA Robot

## Overview
In this project, I have implemented **Position Based Visual Servoing (PBVS)** for the **SCARA** robot. 

---
### Demo 1 - Visual Servoing . . . *in action* !!!
<figure>
    <img src="media/demo1_PBVS.gif" height="360" width="672">
    <figcaption><i>The goal for the SCARA robot is to track the center of the object and hit the target once the end effector stabilizes.</i></figcaption>
</figure>


---
### Demo 2 - Visual Servoing . . . *playback using rosbag* !!!
<figure>
    <img src="media/demo2_rosbag.gif" height="360" width="672" />
    <figcaption><i>Here the original environment is changed by moving the table aside. However, we can make the robot behave as it did before using recorded information through rosbag utility.</i></figcaption>
</figure>

> **Why is this useful?** <br>
*Let's assume that the robot had a problem in the field. In software engineering, one of the best possible way to fix a bug is to be able to replicate the same issue. In the below video, even though the original environment is changed with the table being pushed away, we can still see the image and movements of the robot which will immensely help in fixing the bug in the field.**


---
### Demo 3 - Visual Servoing . . . *plots using rosbag on rqt_gui* !!!  
<figure>
    <img src="media/demo3_plots.gif" height="360" width="672" />
    <figcaption><i>The plots represent the movement of each joint during the visual servoing process. As we can observe, the change is gradual and this shows that the controller I've implemented and the PID gains tuned are doing a pretty good job.</i></figcaption>
</figure>





---
## Deployment Diagram using UML
<figure>
    <img src="media/Deployment_Diagram.png" height="480" width="896" />
    <figcaption><i>The communication between nodes and Gazebo simulator takes place over ROS Topics and ROS Services.</i></figcaption>
</figure>
<br>

Deployment diagrams are a useful tool to get the BIG PICTURE of how the nodes are running and interacting with each other, especially in a multiple process environment like this one. 

The boxes highlighted in the <span style="color:blue">blue</span> are the nodes written by me. The phrases represented in the <span style="color:blue">blue</span> font are the topics and services being provided by the nodes developed by me. 





---
## High Level Design
<figure>
    <img src="media/Logic_BlockDiagram.png" height="480" width="896" />
    <figcaption><i>The following block diagram gives a quick overview of the logic employed to perform Postion Based Visual Servoing for the SCARA robot.</i></figcaption>
</figure>






---
## Tools and Frameworks
**ROS Version:** Noetic

**Gazebo Version:** 11.9.1

**IDE:** Eclipse <br>
*(The required setup was made to use catkin_make to build the roscpp codebase. The necessary environemnt variables were setup to debug them on the fly)*

**3rd party libraries:**
- OpenCV 4.2
- Eigen 3.4.0

**Software Modelling Tool:** Star UML <br>
*(The deployment diagram that describes the modular design of the code and how the different nodes interact with each other was created using this software)*




---
## Contributions
 
- **Robot Description**: <br>
The XACRO file for simulating the robot in Gazebo physics simulator was written from scratch. Further, the xacro file reads the robot's link diemensions from a configuration file called *"scara_config.yaml."* This is to ease the loading of these diemensions to the ROS param server and also to be able to change the robot's size without touching the complex looking xacro file. 
<br><br>

- **Object Description**: <br>
Another XACRO file was written to preserve the modularity of xacro files.
<br><br>

- **ROS Nodes**: <br>

    - **Scara Kinematics**: 
       - <u>*Forward Kinematics*</u>: <br>
        The FK for the scara robot was implemented using the DH table. It was further verified using the tf package using C++. The notes for it has been shown at the bottom of this page.
        
       - <u>*Inverse Kinematics*</u>: <br>
        The IK for the scara robot was implemented using the geometric approach. It was further verified using the FK function.
    	
       - <u>*Jacobian*</u>: <br>
        The Jacobian for the scara robot was implemented.
		<br> <br>
		**Note**: *All of the above have been implemented as ROS Services. These services are used by the Visual Servoing node to perform the PBVS.*
		<br><br>

    - **Visual Servoing using OpenCV**:  
       - <u>*Position Based Visual Servoing*</u>: <br>
        This node processes the raw_image, detects the circle center, computes the change in joint positions for end effector to be centered exactly on the center of the circle and hit the target.
        <br>
        
    - **Visual Servoing using PCL**:     
       - *Possible future work* <br>


- **ROS Plugins**: <br> 

    - **My Velocity Controller**:   
        The interface for the gazebo_ros controller was inheirited to write my <u>own PID controller</u>. The PID values are loaded from the ROS param server. For the integrator part of the controller, an anti-wind up logic was also realized.


- **Gazebo Launch File**: <br>

    - **scara.launch**:   
        This file was written from scratch to bring up the necessary nodes and load the necessary parameters.






---
## Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	```bash
	$ cd catkin_workspace/src
	$ git clone https://github.com/emmanuel-logy/visual_servoing_scara.git
	$ cd ..
	$ catkin_make
	```


---
## Usage

* **STEP 1**: Source your catkin_workspace with 
	
	```bash
	$ source ./devel/setup.bash
	```

* **STEP 2**: Launch the SCARA with the nodes necessary for performing the PBVS
	
	```bash
	$ roslaunch visual_servoing_scara scara.launch
	```

* **[Optional 1]**:

    - **scara_control.yaml** <br>
    This file can be used to vary the PID gains for my custom velocity controller.
    - **scara_config.yaml** <br>
    This file can be used to change the diemensions of the robot on the fly.

* **[Optional 2]**: 
    - **scara_control.yaml** <br>
    To play back the rosbag	
    
    ```bash
	$ rosbag play 2022-01-22-16-27-57.bag /scara/camera1/image_raw:=/rec_vid
	```
	
	
---
    
    
