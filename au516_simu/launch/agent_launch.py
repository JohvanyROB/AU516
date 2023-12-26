from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nb_agents = 1   #TODO: set this variable to 2 when working with 2 agents
    robot_size = 0.5

    agent_nodes = []
    for i in range(nb_agents):
        agent_nodes.append(
            Node(
                package = "au516_nav",
                executable = "agent.py",
                parameters = [
                    {"namespace": f"bot_{i+1}"},
                    {"robot_size": robot_size}, #robot's size/diameter in meter,
                ]
            )
        )

    ld = LaunchDescription()

    for agent_node in agent_nodes[:nb_agents]:
        ld.add_action(agent_node)

    return ld