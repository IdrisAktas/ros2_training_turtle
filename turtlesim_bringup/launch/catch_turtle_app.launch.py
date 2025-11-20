from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   
    ld=LaunchDescription()

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )

    spawn_turtle_node = Node(
        package='turtlesim_py_pkg',
        executable='spawn_turtle_node'
    )

    find_and_kill_node = Node(
        package='turtlesim_py_pkg',
        executable='find_and_kill_node'
    )
    ld.add_action(turtlesim_node)
    ld.add_action(spawn_turtle_node)
    ld.add_action(find_and_kill_node)
    return ld


 