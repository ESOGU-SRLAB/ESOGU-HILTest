import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('ifarlab_description')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'ifarlab_description.urdf')
    
    # --- MODIFICATION START ---
    # Read the URDF file content
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()
    
    # Robot description
    # Pass the content directly
    robot_description = {'robot_description': robot_description_content}
    # --- MODIFICATION END ---
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            # --- MODIFICATION START ---
            robot_description, # Pass the dictionary directly
            # --- MODIFICATION END ---
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # RViz
    rviz_config = os.path.join(pkg_share, 'rviz', 'ifarlab_environment.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-r -v4 ', "default.sdf"], 'on_exit_shutdown': 'true'}.items()
)
    
    start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-topic', "robot_description",
        "-name", "ifarlab_environment",
        '-x', "0",
        '-y', "0",
        '-z', '0.01'
    ],
    output='screen',
)
    gz_to_ros_joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='joint_state_bridge',
        output='screen',
        arguments=[
            # Köprülenecek topic ve mesaj tipleri:
            # Gazebo_Topic@ROS2_Msg_Type@Gazebo_Msg_Type
            '/world/default/model/ifarlab_environment/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '--ros-args', '-p', 'direction:=ROS_TO_GZ' # Köprü yönü: Gazebo'dan ROS2'ye
        ],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        gzserver_cmd,
        start_gazebo_ros_spawner_cmd,
        gz_to_ros_joint_state_bridge
    ])