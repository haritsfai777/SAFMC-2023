"""
Script to move drone based on aruco
"""

#-- LIBRARY IMPORTS --#

import time
import math

# DroneKit
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
from mavlink import *

# WiFi
import socket
import threading

# Force Quit
import sys

# -- Connecting to Leader
# master_connection = '/dev/ttyUSB0'
# master_connection = 'udp:127.0.0.1:14561'
master_connection = '/dev/ttyACM0'

print('Connecting Leader Drone...')
vehicle_master = connect(master_connection, source_system=1, baud=115200) # connect d1 to PX4_1
# vehicle_master = conn
print("Master Connected")

#-- Set up connection to Follower
udp_connection = 'udpin:127.0.0.1:14561'
udp_pipe= MAVConnection(udp_connection, source_system=1)
vehicle_master._handler.pipe(udp_pipe)
udp_pipe.master.mav.srcComponent = 1
udp_pipe.start() # start the pipe to DroneKit_2

while(True):
    print("hi")
    time.sleep(1)
    continue