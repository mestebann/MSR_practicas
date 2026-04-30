from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config = PathJoinSubstitution([
    FindPackageShare("rover_description"),
    "rviz",
    "rover.rviz"
    ])


    rover_description_content = Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("rover_description"), "robots", description_file]),
            " ",
            "prefix:=", 
            prefix,
    ])

    rover_description_param = ParameterValue(rover_description_content, value_type=str)

    rover_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': rover_description_param,
          'publish_frequency': 100.0,
        }],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_publisher_gui),
        parameters=[{
          'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config],
        parameters=[{
        'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "description_file",
            default_value="rover.urdf.xacro",
            description="Archivo XACRO principal del rover"
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefijo para links y joints"
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Usar tiempo de simulación"
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="true",
            description="Lanzar joint_state_publisher_gui"
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Lanzar RViz2"
        ),
        joint_state_publisher_gui_node,
        rover_state_publisher_node,
        rviz_node,
    ])
    