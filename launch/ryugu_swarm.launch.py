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
        # gz-sim auto-generates this long hierarchical topic name for the IMU sensor
        # since the SDF no longer sets an explicit (and incorrectly unscoped)
        # <topic> override -- see generate_detailed_spacehopper.py's IMU sensor
        # comment for the full story (IMU data was silently never reaching
        # attitude_controller.py/landing_controller.py before this fix).
        imu_gz_topic = f'/world/ryugu_world/model/{agent}/link/base_link/sensor/imu_sensor/imu'

        # NOTE on message type strings below: GZ->ROS bridge entries (using the "["
        # bracket) must name the type gz-transport actually publishes on the wire.
        # `gz topic -i -t <topic>` showed these sensor/odometry topics are published
        # as "gz.msgs.X", not "ignition.msgs.X" -- the "ignition.msgs.*" alias isn't
        # registered for IMU/Odometry/LaserScan in this ros_gz_bridge build, so the
        # bridge silently created a dead (non-delivering) subscription for each of
        # them. This is why IMU never reached attitude_controller.py/
        # landing_controller.py, and why swarm_manager.py's odometry-based position
        # tracking looked wired up but was actually always frozen at (0,0). The
        # ROS->GZ command entries below (using "]") were unaffected -- Float64/Double
        # apparently does have that legacy alias -- so they're left as-is since they
        # were already confirmed working (drill, legs, RW respond to commands).
        bridge_config = [
            # IMU
            f'{imu_gz_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # LIDAR
            f'/model/{agent}/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # Odometry (real position/velocity feedback, was previously nonexistent)
            f'/model/{agent}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # Reaction Wheels Velocity
            f'/model/{agent}/joint/rw_x_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            f'/model/{agent}/joint/rw_y_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
            f'/model/{agent}/joint/rw_z_joint/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double',
        ]
        
        # Leg Position Commands
        for j in range(3):
            bridge_config.append(f'/model/{agent}/joint/hip_joint_{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double')
            bridge_config.append(f'/model/{agent}/joint/knee_joint_{j}/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double')

        # Drill/Sampler Position Command (was previously unbridged -- swarm_manager
        # published to cmd_drill but it never reached Gazebo, and the joint had no
        # controller plugin either, so the drill could only passively drift)
        bridge_config.append(f'/model/{agent}/joint/drill_joint/cmd_pos@std_msgs/msg/Float64]ignition.msgs.Double')

        # Create topic remapping rules
        remappings = [
            (imu_gz_topic, f'/{agent}/imu'),
            (f'/model/{agent}/lidar', f'/{agent}/lidar'),
            (f'/model/{agent}/odometry', f'/{agent}/odometry'),
            (f'/model/{agent}/joint/rw_x_joint/cmd_vel', f'/{agent}/rw_x_joint_cmd_vel'),
            (f'/model/{agent}/joint/rw_y_joint/cmd_vel', f'/{agent}/rw_y_joint_cmd_vel'),
            (f'/model/{agent}/joint/rw_z_joint/cmd_vel', f'/{agent}/rw_z_joint_cmd_vel'),
        ]
        for j in range(3):
            remappings.append((f'/model/{agent}/joint/hip_joint_{j}/cmd_pos', f'/{agent}/joint_hip_joint_{j}_cmd_pos'))
            remappings.append((f'/model/{agent}/joint/knee_joint_{j}/cmd_pos', f'/{agent}/joint_knee_joint_{j}_cmd_pos'))
        # Remap straight onto swarm_manager's existing publish topic name
        remappings.append((f'/model/{agent}/joint/drill_joint/cmd_pos', f'/{agent}/cmd_drill'))

        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{agent}',
            arguments=bridge_config,
            remappings=remappings,
            output='screen'
        ))

    return LaunchDescription(nodes)
