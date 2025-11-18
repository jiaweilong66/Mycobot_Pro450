### Local 1. Operation Process
#### 1.1 Installation Prerequisites 

To use this package, you need to install the [Python API](https://github.com/elephantrobotics/pymycobot.git) library first. 

```bash
pip install pymycobot --user
ros1 noetic
gazebo 11
```


#### 1.2 Package Download and Installation 

Download the package into your ROS workspace.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/jiaweilong66/Mycobot_Pro450.git
$ cd ~/catkin_ws
$ catkin_make
$ source devel/setup.bash
```


MyCobot_450_m5-Gazebo User Guide
1. Slider Control
The control of the robot arm model's pose in Gazebo through the sliders of joint_state_publisher_gui has now been achieved. Moreover, the pose of the robot arm model in Gazebo and the real robot arm can be controlled simultaneously via the sliders.
After confirming that the real robot arm is connected to the computer, check the port to which the robot arm is connected: 

```bash
my450 = Pro450Client("192.168.0.232", 4500)
```

The following output results are obtained: 

```bash
```

Open communication and Chmod

```bash
sudo chmod -R 777 mycobotPro450//scripts/follow_display_gazebo.py
sudo chmod -R 777 mycobotPro450/mycobotPro450/scripts/slider_control_gazebo.py
sudo chmod -R 777 mycobotPro450/mycobotPro450/scripts/teleop_keyboard_gazebo.py
roscore
```

After confirming the port, open a terminal and enter the following command. Note that you should replace "port" with the value you found in the previous step. 

```bash
source devel/setup.bash
roslaunch mycobotPro450 slider.launch _port:=/dev/ttyACM0 _baud:=115200
```


Then open another terminal and enter the following command: 

```bash
source devel/setup.bash
rosrun mycobotPro450 slider_control_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


Also remember to modify the port number to the one queried in the previous step. If the operation is successful, you will see the following terminal prompt: 

```bash
('/dev/ttyACM0', 115200)
spin ...
```


At this point, you can control the poses of both the mechanical arm model in Gazebo or the real mechanical arm simultaneously by manipulating the sliders in the joint_state_publisher_gui. 

2. Gazebo Model Following
The following command can be used to make the model in Gazebo change its pose in accordance with the movement of the actual robotic arm. First, run the launch file: 

```bash
source devel/setup.bash
roslaunch mycobotPro450 follower.launch _port:=/dev/ttyACM0
```


If the program runs successfully, the Gazebo interface will successfully load the robotic arm model, and all joints of the robotic arm model will be in the original pose, that is, [0, 0, 0, 0, 0, 0]. After that, we open the second terminal and run: 

```bash
source devel/setup.bash
rosrun mycobotPro450 follow_display_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


Now when we control the pose of the actual robotic arm, we can see that the robotic arm in Gazebo will also move to the same pose together. 

3. Keyboard Control
We can also use keyboard input to simultaneously control the pose of the robotic arm model in Gazebo and the actual robotic arm. First, open a terminal and enter: 

```bash
source devel/setup.bash
roslaunch mycobotPro450 follower.launch _port:=/dev/ttyACM0 _baud:=115200
```


As in the previous part, we will see the robotic arm model loaded into Gazebo, and all joints are at their initial poses. Then we open another terminal and enter: 

```bash
source devel/setup.bash
rosrun mycobotPro450 teleop_keyboard_gazebo.py _port:=/dev/ttyACM0 _baud:=115200
```


If the operation is successful, we will see the following output information in the terminal: 

```shell
mycobotPro450 Teleop Keyboard Controller
---------------------------
Movimg options (control the angle of each joint):
w: joint2_to_joint1++   s: joint2_to_joint1--
e: joint3_to_joint2++   d: joint3_to_joint2--
r: joint4_to_joint3++   f: joint4_to_joint3--
t: joint5_to_joint4++   g: joint5_to_joint4--
y: joint6_to_joint5++   h: joint6_to_joint5--
u: joint6output_to_joint6++ j: joint6output_to_joint6--
o:open gripper          p:close gripper
Other:
1 - Go to home pose
q - Quit
```

You can find out which interfaces pymycobot provides in `README.md`.

Please go to [here](./READMECN.md).


> Note: Version v3.6.0 differentiates interfaces by model. Starting from this version, the MyCobot class will no longer be maintained. For new usage, please refer to the document: 

![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/chinese.svg)   ![jaywcjlove/sb](https://jaywcjlove.github.io/sb/lang/english.svg)

[MyCobot 450m5gazebo中文操作](./READMECN.md)



