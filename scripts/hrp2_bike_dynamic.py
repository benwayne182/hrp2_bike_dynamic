#!/usr/bin/env python

from time import sleep
import subprocess
import sys

terminal = ['gnome-terminal']

# Start roscore in a terminal
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "launch roscore"
roscore
'
''' % locals(), '-t', '''roscore'''])

# Start OpenHRP
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "OpenHRP"
sleep 10
/opt/grx/HRP2LAAS/bin/GrxUI.sh
'
''' % locals(), '-t', '''Start OpenHRP'''])

# Launch openhrp bridge
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "openhrp bridge"
sleep 20
roslaunch hrp2_bike_dynamic hrp2_bike_dynamic.launch
'
''' % locals(), '-t', '''Launch openhrp bridge'''])

# Start dynamic_graph_bridge run_command terminal
terminal.extend(['--tab-with-profile=HoldOnExit', '-e','''
bash -c '
echo "run command :"
sleep 25
rosrun dynamic_graph_bridge run_command
'
''' % locals(), '-t', '''Run Command'''])

# Set the size of the terminal
terminal.extend(['''--geometry=195x50+0+0'''])

subprocess.call(terminal)






