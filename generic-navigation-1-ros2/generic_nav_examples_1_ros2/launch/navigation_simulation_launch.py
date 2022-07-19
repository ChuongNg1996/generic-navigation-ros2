"""
Load Gazebo models without export PATH: https://automaticaddison.com/how-to-load-a-robot-model-sdf-format-into-gazebo-ros-2/
"""

# ----------------------------------------------------------- #
# ------------- STEP 0: IMPORT MODULES/LIBRARIES ------------ #
# ----------------------------------------------------------- #

"""
os - Miscellaneous operating system interfaces LINK: https://docs.python.org/3/library/os.html
This module provides a portable way of using operating system dependent functionality.
+ If you just want to read or write a file see open().
+ If you want to manipulate paths, see the os.path module, ...
"""
import os

"""
get_package_share_directory() - as the name suggests, RETURN path of the package. 
"""
from ament_index_python.packages import get_package_share_directory 

"""
LINK: https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Using-Substitutions.html
DeclareLaunchArgument() is used to define the launch argument that can be passed from the above launch file or from the console.
LaunchConfiguration() substitutions allow us to acquire the value of the launch argument in any part of the launch description.
ExecuteProcess() is used to execute command line
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------------------------------- #
    # ------ STEP 1 (OPTIONAL): Get the LAUNCH DIRECTORIES ------ #
    # ----------------------------------------------------------- #

    navigation_examples_dir = get_package_share_directory('generic_nav_examples_1_ros2')    # Get path of `generic_nav_examples_1_ros2` package, then ...
    navigation_examples_launch_dir = os.path.join(navigation_examples_dir, 'launch')        # ... Join the path with folder `launch`
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros') 

    # ----------------------------------------------------------- #
    # ---- STEP 2 (OPTIONAL): LAUNCH CONFIGURATION VARIABLES ---- #
    # ----------------------------------------------------------- #

    use_simulator = LaunchConfiguration('use_simulator') 
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    environment_sdf_path = LaunchConfiguration('environment_sdf_path')
    robot_sdf_path = LaunchConfiguration('robot_sdf_path')

    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(navigation_examples_dir, 'worlds', 'empty_world.model'),
        description='Full path to world model file to load')
    
    declare_environment_sdf_path_cmd = DeclareLaunchArgument(
        'environment_sdf_path',
        default_value=os.path.join(navigation_examples_dir, 'worlds', 'environment_1.sdf'),
        description='Full path to environment model file to load')
    
    declare_robot_sdf_path_cmd = DeclareLaunchArgument(
        'robot_sdf_path',
        default_value=os.path.join(navigation_examples_dir, 'worlds', 'robot_model.sdf'),
        description='Full path to robot model file to load')


    # ----------------------------------------------------------- #
    # -------------- STEP 3: ROS LAUNCH FILES ------------------- #
    # ----------------------------------------------------------- #

    ################################################################
                    # COMPONENT 2: GAZEBO SIMULATION
    ################################################################

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())
 
  # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
 

    # ----------------------------------------------------------- #
    # -------------------- STEP 4: ROS NODES -------------------- #
    # ----------------------------------------------------------- #

    # Spawn Environment 
    spawn_environment_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'environment', 
               '-file', environment_sdf_path,
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.0',
                  '-Y', '0.0'],
                  output='screen')
    
    # Spawn Robot 
    spawn_robot_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', 'robot', 
               '-file', robot_sdf_path,
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.0',
                  '-Y', '0.0'],
                  output='screen')

    # ----------------------------------------------------------- #
    # --------- STEP 5: Create the LAUNCH DESCRIPTION ----------- #
    # ----------------------------------------------------------- #

    return LaunchDescription([
        declare_use_simulator_cmd,
        declare_simulator_cmd,
        declare_world_cmd,
        declare_environment_sdf_path_cmd,
        declare_robot_sdf_path_cmd,
        
        
        # Run ROS Launch files
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,

        # launch ROS NODES
        spawn_environment_cmd,
        spawn_robot_cmd,

    ])