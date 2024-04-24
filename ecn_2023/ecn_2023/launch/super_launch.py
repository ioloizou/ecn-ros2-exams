from simple_launch import SimpleLauncher

sl = SimpleLauncher()


def launch_setup():

    sl.include(package = 'ecn_usv', launch_file='usv_launch.py', launch_arguments={'Kv': 4., 'Kw': 1.2})

    sl.include(package = 'ecn_2023', launch_file='world_launch.py')
    
    sl.include(package = 'ecn_2023', launch_file='drone_launch.py')

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)