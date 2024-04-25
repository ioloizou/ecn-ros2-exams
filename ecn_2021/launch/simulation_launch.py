from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    
    map_file = sl.find('ecn_2021', 'house.yaml')
    
    # simulation
    sl.node('map_simulator', 'simulator', parameters = {'map': map_file, 'display': False})
    
    # reset poses
    sl.include('ecn_2021', 'reset_launch.py', launch_arguments={'rsp': True})

    # nav for turtlebot1
    sl.include('ecn_2021','turtlebot_nav_launch.py')
            
    # run RViz2
    sl.rviz(sl.find('ecn_2021', 'config.rviz'))
    
    return sl.launch_description()
