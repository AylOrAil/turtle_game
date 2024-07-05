from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )

    turtle_killer_node = Node(
        package='game_controller',
        executable='game_logic',
        name='game_starter_node'
    )

    return LaunchDescription([
        turtlesim_node,
        turtle_killer_node,
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'service', 'call', '/kill', 'turtlesim/srv/Kill', '{name: "turtle1"}'],
                    shell=True
                )
            ]
        )
    ])
