import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ryugu_sim = get_package_share_directory('ryugu_sim')

    # Ensure Gazebo knows where to find the models
    os.environ['GZ_SIM_RESOURCE_PATH'] = '/home/melvin/ryugu_v2_ws/src/ryugu_sim/models'

    # Launch Gazebo Ignition
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', os.path.join(pkg_ryugu_sim, 'worlds', 'ryugu.sdf')],
        output='screen'
    )

    # Launch the Spawner Node (Drops Alpha, Bravo, Charlie)
    spawner = Node(
        package='ryugu_sim',
        executable='spawner',
        name='spawner_node',
        output='screen'
    )

    # Launch the Swarm Intelligence Manager
    swarm_manager = Node(
        package='ryugu_sim',
        executable='swarm_manager',
        name='swarm_manager_node',
        output='screen'
    )

    nodes = [gazebo, spawner, swarm_manager]
    
    # Add Locomotion, Attitude, Landing Controller, and Bridge for a single agent
    for agent in ["scout_1"]:
        nodes.append(Node(
            package='ryugu_sim',
            executable='hopper_locomotion',
            name=f'loco_{agent}',
            arguments=[agent],
            output='screen'
        ))
        
        nodes.append(Node(
            package='ryugu_sim',
            executable='attitude_controller',
            name=f'attitude_{agent}',
            arguments=[agent],
            output='screen'
        ))

        nodes.append(Node(
            package='ryugu_sim',
            executable='landing_controller',
            name=f'landing_{agent}',
            arguments=[agent],
            output='screen'
        ))
        
        # Build the bridge parameters dynamically for this agent
        bridge_config = [
            # IMU
            f'/model/{agent}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            # LIDAR
            f'/model/{agent}/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Reaction Wheels Velocity
            f'/model/{agent}/joint/rw_x_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            f'/model/{agent}/joint/rw_y_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            f'/model/{agent}/joint/rw_z_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
        ]
        
        # Leg Position Commands
        for j in range(3):
            bridge_config.append(f'/model/{agent}/joint/hip_joint_{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double')
            bridge_config.append(f'/model/{agent}/joint/knee_joint_{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double')

        # Create topic remapping rules
        remappings = [
            (f'/model/{agent}/imu', f'/{agent}/imu'),
            (f'/model/{agent}/lidar', f'/{agent}/lidar'),
            (f'/model/{agent}/joint/rw_x_joint/cmd_vel', f'/{agent}/rw_x_joint_cmd_vel'),
            (f'/model/{agent}/joint/rw_y_joint/cmd_vel', f'/{agent}/rw_y_joint_cmd_vel'),
            (f'/model/{agent}/joint/rw_z_joint/cmd_vel', f'/{agent}/rw_z_joint_cmd_vel'),
        ]
        for j in range(3):
            remappings.append((f'/model/{agent}/joint/hip_joint_{j}/cmd_pos', f'/{agent}/joint_hip_joint_{j}_cmd_pos'))
            remappings.append((f'/model/{agent}/joint/knee_joint_{j}/cmd_pos', f'/{agent}/joint_knee_joint_{j}_cmd_pos'))

        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{agent}',
            arguments=bridge_config,
            remappings=remappings,
            output='screen'
        ))

    return LaunchDescription(nodes)
