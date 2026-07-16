import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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

    # Swarm status dashboard (role/activity per bot, leg/drill/RW telemetry,
    # attitude gyro indicator) -- see swarm_gui.py.
    swarm_gui = Node(
        package='ryugu_sim',
        executable='swarm_gui',
        name='swarm_gui_node',
        output='screen'
    )

    # Auto window layout: sim window at the left 3/4 of the screen, dashboard
    # docked at the right 1/4, so the user doesn't have to manually arrange
    # them every launch. Runs after a delay so both windows have actually
    # appeared (Gazebo's GUI window in particular takes a few seconds).
    # wmctrl -e format is "gravity,x,y,width,height"; screen assumed 1920x1080
    # (this machine's actual resolution -- there's no clean way to query the
    # target display's resolution from inside a ROS 2 launch action, so this
    # is hardcoded rather than dynamically detected).
    layout_windows = TimerAction(
        period=10.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'wmctrl -r "Gazebo Sim" -b remove,maximized_vert,maximized_horz; '
                 'wmctrl -r "Gazebo Sim" -e 0,0,0,1440,1080; '
                 'wmctrl -r "Ryugu Swarm Dashboard" -b remove,maximized_vert,maximized_horz; '
                 'wmctrl -r "Ryugu Swarm Dashboard" -e 0,1440,0,480,1080'],
            output='screen'
        )]
    )

    nodes = [gazebo, spawner, swarm_manager, swarm_gui, layout_windows]
    
    # Locomotion, Attitude, Landing Controller, and Bridge per agent
    # (scaled to the full 3-bot swarm 2026-07-16 -- keep in sync with
    # spawner.py's AGENTS list and swarm_manager.py's self.agents)
    for agent in ["scout_1", "scout_2", "scout_3"]:
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
        
        # Bridge config via YAML file (rewritten 2026-07-16). WHY: gz-sim 8's
        # JointPositionController subscribes ONLY to the joint-INDEXED topic
        # /model/<m>/joint/<j>/0/cmd_pos (verbose-server-verified), and ROS
        # remap rules cannot express a numeric path token ("/0/"), so the
        # old CLI-args + remappings approach could not reach the plugin at
        # all. The YAML config decouples gz_topic_name (not ROS-restricted)
        # from ros_topic_name. The old un-indexed bridge SILENTLY published
        # into the void for position controllers.
        imu_gz_topic = f'/world/ryugu_world/model/{agent}/link/base_link/sensor/imu_sensor/imu'
        entries = [
            (f'/{agent}/imu', imu_gz_topic,
             'sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS'),
            (f'/{agent}/odometry', f'/model/{agent}/odometry',
             'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        ]
        for axis in ['x', 'y', 'z']:
            entries.append((f'/{agent}/rw_{axis}_joint_cmd_vel',
                            f'/model/{agent}/joint/rw_{axis}_joint/cmd_vel',
                            'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
        for j in range(3):
            for jt in ['hip', 'knee']:
                entries.append((f'/{agent}/joint_{jt}_joint_{j}_cmd_pos',
                                f'/model/{agent}/joint/{jt}_joint_{j}/0/cmd_pos',
                                'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))
        entries.append((f'/{agent}/cmd_drill',
                        f'/model/{agent}/joint/drill_joint/0/cmd_pos',
                        'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'))

        cfg_path = f'/tmp/ryugu_bridge_{agent}.yaml'
        with open(cfg_path, 'w') as f:
            for ros_t, gz_t, ros_ty, gz_ty, dr in entries:
                f.write(f'- ros_topic_name: "{ros_t}"\n'
                        f'  gz_topic_name: "{gz_t}"\n'
                        f'  ros_type_name: "{ros_ty}"\n'
                        f'  gz_type_name: "{gz_ty}"\n'
                        f'  direction: {dr}\n')

        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{agent}',
            parameters=[{'config_file': cfg_path}],
            output='screen'
        ))

    return LaunchDescription(nodes)
