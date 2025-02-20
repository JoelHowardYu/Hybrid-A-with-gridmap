from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the package share directory
    create_maps_path = get_package_share_directory('create_maps')

    # Define the path to the config file
    config_file_path = LaunchConfiguration(
        'config_file_path',
        default=[create_maps_path, '/config/simple_demo.yaml']
    )

    return LaunchDescription([
        # Launch the grid map simple demo node
        Node(
            package='create_maps',
            executable='simple_demo_node',
            name='grid_map_simple_demo',
            output='screen'
        ),
        Node(
            package='create_maps',
            executable='hybrid_astar_test',
            name='hybrid_astar_test',
            output='screen'
        ),
        # Uncomment the following node if you want to launch the ompl_test_node
        # Node(
        #     package='create_maps',
        #     executable='ompl_test',
        #     name='ompl_test',
        #     output='screen'
        # ),

        # Launch the grid map visualizer
        # Node(
        #     package='grid_map_visualization',
        #     executable='grid_map_visualization',
        #     name='grid_map_visualization',
        #     output='screen',
        #     parameters=[config_file_path]
        # ),

        # Launch RViz with the demo configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', [create_maps_path, '/rviz/map.rviz']]
        )
    ])