from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
import os

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    # ZED camera arguments
    camera_name_param = DeclareLaunchArgument(
        'camera_model', default_value='zed2i', description='Camera name'
    )


    shadow_directory = get_package_share_directory('shadow')
    config_helios_param = DeclareLaunchArgument(
        'config_helios',
        default_value=os.path.join(shadow_directory, 'config', 'config_helios.yaml'),
        description='Ruta del archivo de configuración del LIDAR 1 (Helios)'
    )

    # Argumento de lanzamiento para la configuración del LIDAR 2 (BPearl)
    config_bpearl_param= DeclareLaunchArgument(
        'config_bpearl',
        default_value=os.path.join(shadow_directory, 'config', 'config_bpearl.yaml'),
        description='Ruta del archivo de configuración del LIDAR 2 (BPearl)'
    )
    # # Pointcloud to LaserScan arguments
    # cloud_in_arg = DeclareLaunchArgument(
    #     'cloud_in', default_value='rslidar_points', description='Cloud in topic'
    # )
    # min_height_arg = DeclareLaunchArgument(
    #     'min_height', default_value='-0.01', description='Min pointcloud height'
    # )
    # max_height_arg = DeclareLaunchArgument(
    #     'max_height', default_value='0.01', description='Max pointcloud height'
    # )

        # Ruta al archivo XACRO
    xacro_file = PathJoinSubstitution([
        FindPackageShare('shadow'),
        'urdf',
        'shadow_description.urdf'  # Nombre del archivo XACRO
    ])

    # Configuración del parámetro robot_description
    robot_description = Command(['xacro ', xacro_file])

    joint_state_publisher_node = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        )
    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': launch_ros.descriptions.ParameterValue(robot_description, value_type=str)  }],
        )
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )

    # Joystick node
    joystick_node = Node(
            package="joy",
            executable="joy_node",
            output="screen",
        )
    
    # ZED camera
    zed_ros2_wrapper = get_package_share_directory("zed_wrapper")
    zed_launch_file = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([zed_ros2_wrapper, "launch", "zed_camera.launch.py"])
            ),
            launch_arguments={'camera_model': LaunchConfiguration('camera_model')}.items()
        )

    ricoh_theta_z1_node = Node(
            package="camera_360",  
            executable="ricoh_theta_z1",
            output="screen",
        )

    # Nodo para el LIDAR 1 (Helios)
    helios_node = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="Helios",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('config_helios')}]
    )

    # Nodo para el LIDAR 2 (BPearl)
    bpearl_node = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="BPearl",
        output="screen",
        parameters=[{'config_path': LaunchConfiguration('config_bpearl')}]
    )
    
    pointcloud_to_scan_node = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name='pointcloud_to_laserscan_node',
            # parameters=[
            #     {'min_height': -0.01},
            #     {'max_height': 0.01}
            #             ],
            remappings=[
            ('/cloud_in', '/rslidar_points')
                        ]
        )


    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node,
        config_helios_param,
        config_bpearl_param,
        helios_node,
        bpearl_node,
        pointcloud_to_scan_node,
        ricoh_theta_z1_node,
        camera_name_param,
        zed_launch_file,
        rviz_node

    ])

