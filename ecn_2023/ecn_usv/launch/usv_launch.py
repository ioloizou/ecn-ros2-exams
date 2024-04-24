from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher()
    sl.declare_arg('Kv', 3.)
    sl.declare_arg('Kw', .5)

    with sl.group(ns = 'usv'):
        sl.robot_state_publisher('ecn_usv', 'usv.xacro')

        sl.node('ecn_usv', 'usv_sim.py', parameters={'Kv': sl.arg('Kv'), 'Kw': sl.arg('Kw')})
        sl.node('ecn_usv', 'trajectory.py')

    return sl.launch_description()
