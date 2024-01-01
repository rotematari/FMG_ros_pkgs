#! usr/bin/env python3  
from launch.conditions import IfCondition, UnlessCondition
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument ,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_ip_parameter_name = 'robot_ip'
    load_gripper_parameter_name = 'load_gripper'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    use_rviz_parameter_name = 'use_rviz'
    use_joint_state_gui_parameter_name = 'use_gui'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    use_rviz = LaunchConfiguration(use_rviz_parameter_name)
    use_joint_state_gui = LaunchConfiguration(use_joint_state_gui_parameter_name)
    

    franka_bringup_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('franka_bringup'), 'launch'),
         'franka.launch.py']),
         launch_arguments= {robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware},
         condition=IfCondition(use_fake_hardware), 
      )
        



    return LaunchDescription([
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            default_value='169.254.203.254',
            description='Hostname or IP address of the robot.'),
        DeclareLaunchArgument(
            use_rviz_parameter_name,
            default_value='True',
            description='Visualize the robot in Rviz'),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='True',
            description='Use fake hardware'),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when '{}' is true".format(
                use_fake_hardware_parameter_name)),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as an end-effector, otherwise, the robot is loaded '
                        'without an end-effector.'),
        DeclareLaunchArgument(
            use_joint_state_gui_parameter_name,
            default_value='false',
            description='use GUI to controll the human'),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('franka_bringup'), 'launch'),
         '/franka.launch.py']),
         launch_arguments= {robot_ip_parameter_name: robot_ip,
                              use_fake_hardware_parameter_name: use_fake_hardware,
                              use_rviz_parameter_name: use_rviz ,
                              fake_sensor_commands_parameter_name:fake_sensor_commands,
                              load_gripper_parameter_name:load_gripper,
                              }.items(),
         condition=IfCondition(use_fake_hardware), 
        ),

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('urdf_publisher'), 'launch'),
         '/table.launch.py']),        
        ),
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('urdf_publisher'), 'launch'),
         '/human.launch.py']),
         launch_arguments={use_joint_state_gui_parameter_name:use_joint_state_gui,}.items(),        
        ),
])