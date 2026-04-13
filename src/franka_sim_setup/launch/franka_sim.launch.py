import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Paths
    pkg_franka_sim_setup = get_package_share_directory('franka_sim_setup')
    pkg_franka_description = get_package_share_directory('franka_description')
    
    # Set Gazebo Resource Path
    # This allows Gazebo to find meshes in the install directory
    # Gazebo looks for model://franka_description, so we need the directory containing the 'franka_description' folder
    description_share_dir = os.path.dirname(pkg_franka_description)
    
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[pkg_franka_sim_setup, ':', description_share_dir]
    )
    
    # World file
    world_file = os.path.join(pkg_franka_sim_setup, 'worlds', 'pick_and_place.sdf')
    
    # 1. URDF/Xacro
    xacro_file = os.path.join(pkg_franka_sim_setup, 'robots', 'fer_with_camera.urdf.xacro')
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ", xacro_file, " ",
            "gazebo:=true ",
            "ros2_control:=true ",
            "hand:=true ",
            "robot_namespace:='' ",
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 2. SRDF
    srdf_file = os.path.join(pkg_franka_description, 'robots', 'fer', 'fer.srdf.xacro')
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"), " ", srdf_file, " ",
            "hand:=true ",
            "robot_type:=fer ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # 3. Kinematics
    kinematics_yaml = load_yaml(os.path.join(pkg_franka_sim_setup, 'config', 'moveit', 'kinematics.yaml'))
    
    # 4. OMPL Planning Pipeline
    ompl_planning_yaml = load_yaml(os.path.join(pkg_franka_sim_setup, 'config', 'moveit', 'ompl_planning.yaml'))
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                'default_planner_request_adapters/ResolveConstraintFrames '
                                'default_planner_request_adapters/FixWorkspaceBounds '
                                'default_planner_request_adapters/FixStartStateBounds '
                                'default_planner_request_adapters/FixStartStateCollision '
                                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # 5. Controllers Configuration
    moveit_controllers = load_yaml(os.path.join(pkg_franka_sim_setup, 'config', 'moveit', 'moveit_controllers.yaml'))

    # 6. Trajectory Execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # 7. Joint Limits
    joint_limits_path = os.path.join(pkg_franka_sim_setup, 'config', 'moveit', 'joint_limits.yaml')
    joint_limits_yaml = { 'robot_description_planning': load_yaml(joint_limits_path)}

    # 8. Planning Scene Monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fer',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen',
    )

    # Controller Manager configuration file
    controller_config_file = os.path.join(pkg_franka_sim_setup, 'config', 'franka_controllers.yaml')

    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    franka_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_arm_controller", "--controller-manager", "/controller_manager",
                   "--param-file", controller_config_file],
    )

    franka_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_gripper_controller", "--controller-manager", "/controller_manager",
                   "--param-file", controller_config_file],
    )

    # GZ Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/wrist_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/wrist_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/wrist_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/detachable_joint/attach@std_msgs/msg/String@gz.msgs.StringMsg',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/gripper_left_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/gripper_right_contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/overhead_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/overhead_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/overhead_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        ],
        output='screen'
    )


    # RViz2

    # Object Detector (Wrist)
    wrist_detector = Node(
        package='franka_sim_setup',
        executable='object_detector.py',
        name='wrist_detector',
        parameters=[
            {'image_topic': '/wrist_camera/image'},
            {'output_topic': '/detected_object_pixel'},
            {'camera_name': 'wrist'},
            {'use_sim_time': True}
        ]
    )

    # Global Overhead Detector
    overhead_detector = Node(
        package='franka_sim_setup',
        executable='overhead_detector.py',
        name='overhead_detector',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    # MoveIt 2: MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            joint_limits_yaml,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # RViz2
    rviz_config_file = os.path.join(pkg_franka_sim_setup, 'rviz', 'franka_sim.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            joint_limits_yaml,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        joint_state_broadcaster_spawner,
        franka_arm_controller_spawner,
        franka_gripper_controller_spawner,
        wrist_detector,
        overhead_detector,
        move_group_node,
        rviz2,
    ])
