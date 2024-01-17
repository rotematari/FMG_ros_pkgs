import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    
    use_joint_state_gui_parameter_name = 'use_gui'
    use_joint_state_gui = LaunchConfiguration(use_joint_state_gui_parameter_name)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # print(use_joint_state_gui)
    # use_joint_state_gui = True
    urdf_file_name = 'humanSubject01_48dof.urdf'
    urdf = os.path.join(
        get_package_share_directory('urdf_publisher'),
        urdf_file_name)
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            use_joint_state_gui_parameter_name,
            default_value='True',
            description='use GUI to controll the human'),
        Node(
            package='robot_state_publisher',
            namespace = 'human',
            executable='robot_state_publisher',
            name='human_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace = 'human',
            name='joint_state_publisher',
            parameters=[
                {
                    'source_list': ['human/joint_states'],
                 'rate': 30}],
            # condition=UnlessCondition(use_joint_state_gui),
        ),
        # Node(
        #     package='joint_state_publisher_gui',
        #     namespace = 'human',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     parameters=[
        #         {
        #             'source_list': ['human/joint_states'],
        #          'rate': 30}],
        #     condition=IfCondition(use_joint_state_gui),
        #     ),
        

    ])