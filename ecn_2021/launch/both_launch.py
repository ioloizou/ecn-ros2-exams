from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    # TODO run your node for turtlebot2 and turtlebot3 with different distances
    with sl.group(ns='/turtlebot2'):
        sl.node(package='ecn_2021', executable='control', parameters={'distance': 1., 'robot_name': 'turtlebot2'})
    
    sl.node(package='ecn_2021', executable='control',
            parameters = {'distance': 0.8, 'robot_name': 'turtlebot3'},
            remappings = {'/scan': '/turtlebot3/scan', '/cmd_vel': '/turtlebot3/cmd_vel'})

    return sl.launch_description()
