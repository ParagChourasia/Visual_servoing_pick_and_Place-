import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paths
    pkg_franka_description = get_package_share_directory('franka_description')
    pkg_rosbot_xl_description = get_package_share_directory('rosbot_xl_description')
    pkg_husarion_components = get_package_share_directory('husarion_components_description')
    pkg_realsense = get_package_share_directory('realsense2_description')
    
    # ROSbot spawn arguments
    x_arg = DeclareLaunchArgument('x', default_value='-4.0')
    y_arg = DeclareLaunchArgument('y', default_value='-4.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.1')
    namespace_arg = DeclareLaunchArgument('namespace', default_value='rosbot')
    use_sim_arg = DeclareLaunchArgument('use_sim', default_value='True')

    # Environment variables for meshes
    resource_path_values = os.path.join(pkg_franka_description, '..') + ':' + \
                           os.path.join(pkg_rosbot_xl_description, '..') + ':' + \
                           os.path.join(pkg_husarion_components, '..') + ':' + \
                           os.path.join(pkg_realsense, '..') + ':' + \
                           '/opt/ros/humble/share'

    gz_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path_values
    )
    ign_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path_values
    )

    # ROSbot Spawn logic
    spawn_rosbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbot_xl_gazebo'), 'launch', 'spawn.launch.py')
        ),
        launch_arguments={
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'namespace': LaunchConfiguration('namespace'),
            'launch_clock_bridge': 'False',
            'use_sim': LaunchConfiguration('use_sim'),
        }.items(),
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        use_sim_arg,
        gz_resource_path,
        ign_resource_path,
        spawn_rosbot,
    ])
