import os, xacro
from random import uniform

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("au516_desc"), "xacro", "robot.xacro")
    base_colors = ("Red", "Yellow")

    robots = [
        {"name" : "bot_1", "x" : uniform(-3, 3), "y" : uniform(-3, 3), "yaw": uniform(-1.57, 1.57)},
        {"name" : "bot_2", "x" : uniform(-3, 3), "y" : uniform(-3, 3), "yaw": uniform(-1.57, 1.57)}
    ]

    nb_agents = 2

    spawn_robots_cmds = []
    for robot in robots[:nb_agents]:
        spawn_robots_cmds.append(
            Node(
                package = "gazebo_ros",
                executable = "spawn_entity.py",
                arguments = [
                    "-entity", robot["name"], 
                    "-topic", f"/{robot['name']}/robot_description",
                    "-x", str(robot["x"]), 
                    "-y", str(robot["y"]),
                    "-Y", str(robot["yaw"]),
                    "-robot_namespace", robot["name"]
                ]
            )         
        )
    
    robot_state_pub_cmds = []
    for i, robot in enumerate(robots[:nb_agents]):
        robot_state_pub_cmds.append(
             Node(
                package = "robot_state_publisher",
                executable = "robot_state_publisher",
                namespace = robot["name"],
                parameters = [
                    {"robot_description": xacro.process_file(xacro_file, mappings={"base_color_arg": base_colors[i]}).toxml()},
                    {"frame_prefix" : f"{robot['name']}/"}
                ]
            )
        )


    #*************************************************************    
    ld = LaunchDescription()

    for spawn_robot in spawn_robots_cmds[:nb_agents]:
        ld.add_action(spawn_robot)
    
    for robot_state_pub in robot_state_pub_cmds[:nb_agents]:
        ld.add_action(robot_state_pub)
    
    
    return ld