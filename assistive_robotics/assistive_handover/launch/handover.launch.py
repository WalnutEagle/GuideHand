from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # This is the ONE part you MUST change.
    # Get these values by running a hand-eye calibration (e.g., with an ArUco tag)
    # These values are (X, Y, Z, Roll, Pitch, Yaw)
    # 'base_link' -> 'oak_d_camera_optical_frame'
    camera_calibration_args = [
        '-0.1',    # X: PLACEHOLDER
        '0.0',     # Y: PLACEHOLDER
        '0.3',     # Z: PLACEHOLDER
        '0.0',     # Roll: PLACEHOLDER
        '0.0',     # Pitch: PLACEHOLDER
        '0.0',     # Yaw: PLACEHOLDER
        'base_link',
        'oak_d_camera_optical_frame'
    ]

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='vx300s',
        description='Robot model name'
    )
    robot_model = LaunchConfiguration('robot_model')

    # 1. Launch the Interbotix MoveIt Configuration
    # (This is what you installed with the xsarm_amd64_install.sh script)
    interbotix_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(PathJoinSubstitution([robot_model, '_moveit_config'])),
                'launch',
                'xsarm_moveit.launch.py'
            ])
        ),
        launch_arguments={
            'robot_model': robot_model,
            'hardware_type': 'actual', # Use 'fake' for simulation
            'use_rviz': 'true',
            'dof': '5',
        }.items()
    )
    
    # 2. Launch the Static Transform (Your Calibration)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_tf',
        arguments=camera_calibration_args,
        output='screen',
    )
    
    # 3. Launch our "Off-the-Shelf" Perception Node
    perception_node = Node(
        package='assistive_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
    )
    
    # 4. Launch our Handover Manager Node
    handover_manager_node = Node(
        package='assistive_handover',
        executable='handover_manager',
        name='handover_manager',
        output='screen',
        parameters=[{'robot_model': robot_model}]
    )
    
    return LaunchDescription([
        robot_model_arg,
        interbotix_moveit_launch,
        static_tf_node,
        perception_node,
        handover_manager_node
    ])