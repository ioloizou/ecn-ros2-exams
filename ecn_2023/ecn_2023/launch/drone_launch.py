from simple_launch import SimpleLauncher

sl = SimpleLauncher()


def launch_setup():

   
    for i in (1,2,3,4):
        with sl.group(ns = f'uav{i}'):
            # Make a string with the arguments    
            tf_args = ["10 0 15 0 0 0 usv/base_link uav1/target",
                       "0 10 15 0 0 0 usv/base_link uav2/target",
                       "-10 0 15 0 0 0 usv/base_link uav3/target",
                       "0 -10 15 0 0 0 usv/base_link uav4/target"]
            
            # Start the node tf node
            sl.node(package='tf2_ros', executable='static_transform_publisher', arguments=tf_args[i-1].split())

            # Start the controller node
            sl.node('ecn_2023', 'uav', parameters={'drone_number': str(i)}, output="screen")

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)