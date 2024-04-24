#!/usr/bin/env python3

from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    
    for side in ('left','right'):
        with sl.group(ns=side):
            sl.include('ecn_2022','slider_launch.py')
        sl.node('ecn_2022', 'control_node',name=side+'_control', parameters = {'arm': side})

    return sl.launch_description()