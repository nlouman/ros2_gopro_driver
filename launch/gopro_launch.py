# from launch import LaunchDescription
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os

# def generate_launch_description():
#     # Find the config file in the installed share directory
#     pkg_share = get_package_share_directory('ros2_gopro_driver')
#     config_path = os.path.join(pkg_share, 'config', 'gopro_config.yaml')

#     return LaunchDescription([
#         Node(
#             package='ros2_gopro_driver',
#             executable='gopro_node',
#             name='gopro_node',
#             parameters=[config_path],
#             output='screen'
#         )
#     ])

import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Define the configurable parameters for GoPro
configurable_parameters = [
    {"name": "camera_name", "default": "gopro_camera", "description": "Camera unique name"},
    {"name": "camera_namespace", "default": "gopro", "description": "Namespace for the camera"},
    {"name": "serial_no", "default": "''", "description": "Choose device by serial number"},
    {"name": "usb_port_id", "default": "''", "description": "Choose device by USB port ID"},
    {"name": "frame_id", "default": "gopro_link", "description": "Frame ID for TF"},
    {"name": "config_file", "default": "/gopro_config.yaml", "description": "GoPro configuration YAML file"},
    {"name": "log_level", "default": "INFO", "description": "Log level [DEBUG|INFO|WARN|ERROR|FATAL]"},
    {"name": "enable_rgb", "default": "true", "description": "Enable RGB stream"},
    {"name": "enable_depth", "default": "false", "description": "Enable depth stream"},
    {"name": "output", "default": "screen", "description": "Output options [screen|log]"},
]

# Declare the parameters for the launch file
def declare_configurable_parameters(parameters):
    return [
        DeclareLaunchArgument(param["name"], default_value=param["default"], description=param["description"])
        for param in parameters
    ]

# Set up the parameters for the node
def set_configurable_parameters(parameters):
    return dict([(param["name"], LaunchConfiguration(param["name"])) for param in parameters])

# Helper function to load the YAML configuration file
def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

# Function that handles the node setup
def launch_setup(context, params, param_name_suffix=""):
    _config_file = LaunchConfiguration("config_file" + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    _output = LaunchConfiguration("output" + param_name_suffix)
    if os.getenv("ROS_DISTRO") == "foxy":
        # For ROS Foxy, output needs to be handled differently
        _output = context.perform_substitution(_output)

    return [
        launch_ros.actions.Node(
            package="ros2_gopro_driver",  # Your GoPro driver package
            namespace=params_from_file["camera_namespace"],  # Namespace
            name=params_from_file["camera_name"],  # Node name
            executable="gopro_node",  # The executable for your GoPro node
            parameters=[params, params_from_file],  # Parameters from launch arguments
            output=_output,
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level" + param_name_suffix)],  # Logging level
            emulate_tty=True,
        )
    ]

# Generate the launch description
def generate_launch_description():
    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
            OpaqueFunction(function=launch_setup, kwargs={"params": set_configurable_parameters(configurable_parameters)})
        ]
    )