import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Launch the Swarm Intelligence Manager
    swarm_manager = Node(
        package='ryugu_sim',
        executable='swarm_manager',
        name='swarm_manager_node',
        output='screen'
    )

    nodes = [swarm_manager]
    
    # Add Locomotion Controllers for each agent
    for agent in ["scout_1", "scout_2", "scout_3"]:
        nodes.append(Node(
            package='ryugu_sim',
            executable='hopper_locomotion',
            name=f'loco_{agent}',
            arguments=[agent],
            output='screen'
        ))

    return LaunchDescription(nodes)
