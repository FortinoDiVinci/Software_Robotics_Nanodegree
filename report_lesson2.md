![udacity logo](misc/udacity_logo.png)
# Project: Kinematics Pick & Place
### Vincent FORTINEAU, R&D Engineer at ALPhANOV, Bordeaux, France

#### Project due : 23rd May 2018
---


##Goals and steps of the project

1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc/misc1.png
[image2]: ./misc/misc3.png
[image3]: ./misc/misc2.png
[DH_schema]: ./misc/schema_DH.png

####[Evaluation criteria](https://review.udacity.com/#!/rubrics/972/view)  

## Summary

* [Kinematic Analysis](#part1)
	* [DH Parameters](#1-1)
	* [Transformation Matrices](#1-2)
	* [Inverse Kinematics](#1-3)
		* [Inverse Position Kinematics](#1-3-a)
		* [Inverse Orientation Kinematics](#1-3-b)
* [Project Implementation](#part2)
	* [Script implementation](#2-1)
	* [Results](#2-2)

### Kinematic Analysis <a name="part1"></a>
#### 1. DH Parameters <a name="1-1"></a>

The Denavit Hartenberg (DH) representation of the Kuka KR210 is shown bellow :

![Denavit-Hartenberg representation][DH_schema]

To find out the values of the constant $$\inline a_{n-1}$$, $$\inline d_{n}$$ and $$\inline \alpha_{n-1} $$, we need to match the URDF file provided for the kr210 and our DH representation. 

Joint name | Parent link | Child link | x   | y   | z 
---        | ---         | ---        | --- | --- | ---
joint_1    | base_link   | link_1     | 0   |  0  | 0.33
joint_2    | link_1      | link_2     | 0.35|  0  | 0.42
joint_3    | link_2      | link_3     | 0   |  0  | 1.25
joint_4    | link_3      | link_4     | 0.96|  0  | -0.054
joint_5    | link_4      | link_5     | 0.54|  0  | 0
joint_6    | link_5      | link_6     |0.193|  0  | 0
gripper_joint | link_6   | gripper_link|0.11|  0  | 0

In the URDF file, each joint is defined according to its parent. The reference of its parent is set to the joint center and not to the joint origin. (eg. joint 1 origin is shown on the upper figure as $$\inline O_{1}$$, but in the URDF file, the center of the joint is different).

Links| alpha(i-1) | a(i-1) | d(i-1) | theta(i)
---  | --- | --- | --- | ---
0->1 | 0    | 0    | 0.33 + 0.42 | q1
1->2 | -90° | 0.35 | 0      | q2 - 90° 
2->3 | 0    | 1.25 | 0      | q3
3->4 | -90° |-0.054| 0      | q4
4->5 |  90° | 0 |0.96 + 0.54| q5
5->6 | -90° | 0    | 0      | q6
6->G | 0    | 0 |0.193 + 0.11| q7 = 0°


#### 2. Transformation Matrices <a name="1-2"></a>



#### 3. Inverse Kinematics <a name="1-3"></a>

####Inverse Position Kinematics <a name="1-3-a"></a>
 
And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]
 
####Inverse Orientation Kinematics <a name="1-3-b"></a>


### Project Implementation <a name="part2"></a>

#### 1. `IK_server.py` script implementation <a name="2-1"></a>

#### 2. Results <a name="2-2"></a>
Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


