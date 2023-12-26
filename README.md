# AU516 ROS project

For this project, you will work with two simulated robots that have to follow a blue ball, as explained in the previous project.
They have a front-facing camera whose images will be processed by a python script in order to send the adequate velocity commands to each robot.

## Create a ROSject
Connect to [RDS](https://app.theconstructsim.com/#/) and enter your username and password if needed.

On the left panel, click on **My Rosjects** ,then create a new one:

* ROS Distro: **ROS Galactic**
* Name: *AU516_IPSA* (for instance)
* Description: *This is my project for AU516 project* (for instance)

Then click on **Create** and run the ROSject.



## Clone the Github repository
On ROS Development Studio, open a terminal (Web shell) and follow the instructions **ONE AFTER THE OTHER**:
```bash
cd ~/ros2_ws
git clone https://github.com/JohvanyROB/AU516
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```


## Install dependencies
Open a terminal and run the following instructions
```bash
sudo apt update
pip3 install transforms3d
sudo apt install -y ros-galactic-xacro ros-galactic-tf2-tools ros-galactic-tf-transformations
```


## Run the simulation
Open a terminal and run the following instruction:
```bash
ros2 launch au516_simu start_world_launch.py
```
Two robots should appear in the environment at random positions in the range $[-3, 3, 0]^{T}$. A blue ball will be located at $[0, 0, 0.3]^{T}$.


## Control the agents with a python script
To control each agent, the same python script will be edited. Here is the path to access it using the Code Editor on ROS Development Studio:
<p align="center">~/ros2_ws/src/AU516/au516_nav/au516_nav/agent.py</p>

- In the python script, the name of the current robot that runs it will be either **bot_1** or **bot_2**. It is stored in the attribute **self.ns**.
- Whenever a new image is available for the specific robot, the method **image_cb** is called. This is where you should add your code.
- To send velocity commands to the current robot that runs the script, call the method **send_velocities** and set the linear and angular velocities in the range [-1.0, 1.0].
- To set the number of agents that will be controlled with the script, change the value of the variable **nb_agents** to 1 or 2 in the file located at:
<p align="center">~/ros2_ws/src/AU516/au516_simu/agent_launch.py</p>

To run the agent node, **open a second terminal** and run:
```bash
ros2 launch au516_simu agent_launch.py
```

## Move the blue ball in the simulator
On Gazebo, click on the blue ball and use the **translation mode** in the toolbar, as you did during the *Drone and visual servoing project*.