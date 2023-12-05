import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    description_package_name = "barista_robot_description"
    
   
   # Include the Gazebo launch file with the modified launch arguments
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('barista_robot_description'), 'launch'),
        '/gazebo.launch.py'])
    )

    # Define the robot model files to be used
    robot_desc_file = "barista_robot_model.urdf.xacro"
    robot_desc_path = os.path.join(get_package_share_directory(
        "barista_robot_description"), "xacro", robot_desc_file)

    robot_name_1 = "robot1"
    robot_name_2 = "robot2"

    rsp_robot1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_1,
        parameters=[{'frame_prefix': robot_name_1+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_1, ' include_laser:=True'])}],
        output="screen"
    )

    rsp_robot2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name_2,
        parameters=[{'frame_prefix': robot_name_2+'/', 'use_sim_time': True,
                     'robot_description': Command(['xacro ', robot_desc_path, ' robot_name:=', robot_name_2, ' include_laser:=True'])}],
        output="screen"
    )

    spawn_robot1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_1, '-x', '0.0', '-y', '0.0', '-z', '0.0',
                   '-topic', robot_name_1+'/robot_description']
    )

    spawn_robot2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_2, '-x', '1.0', '-y', '1.0', '-z', '0.0',
                   '-topic', robot_name_2+'/robot_description']
    )

    st_pub_robot1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_robot1_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', '/robot1/odom']
    )

    st_pub_robot2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_robot2_odom',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', '/robot2/odom']
    )

    joint_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
        )


     # RVIZ config
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', 'rviz2.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
        )

    return LaunchDescription([
        gazebo_world,
        rsp_robot1,
        rsp_robot2,
        spawn_robot1,
        spawn_robot2,
        rviz_node,
        st_pub_robot1,
        st_pub_robot2
    ])