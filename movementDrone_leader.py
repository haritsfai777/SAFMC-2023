"""
Script to move drone based on aruco
"""

"""
HOW TO RUN

1. Make udp port in both leader and follow
In Leader odroid:
mavproxy.py --master=/dev/ttyACM0 --out udp:192.168.0.102:14560 --out udp:192.168.0.104:14550
mavproxy.py --master=/dev/ttyACM0 --out udp:192.168.0.102:14570 --out udp:192.168.0.105:14550

2. Run t265_to_mavlink.py in both drone

3. Run newMain.py in leader

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

# T265
# from t265_to_mavlink import *

#-- Connect to the vehicle
# print('Connecting Master Drone...')
# vehicle_master = connect('udp:127.0.0.1:14571')

# print('Connecting Slave Drone...')
# vehicle_slave = connect('udp:127.0.0.1:14561')

with_follower = True
using_sitl = True

#-- Connecting to Leader
if (using_sitl):
    master_connection = 'udp:127.0.0.1:14561'

    if (with_follower):
        slave_connection = 'udp:127.0.0.1:14571'
else:
    master_connection = 'udp:192.168.0.102:14560'

    if (with_follower):
        slave_connection = 'udp:192.168.0.102:14570'

print('Connecting Leader Drone...')
vehicle_master = connect(master_connection, source_system=1)

if (with_follower):
    print('Connecting Follower Drone...')
    vehicle_slave = connect(slave_connection, source_system=1)

#-- Set up connection to Follower
#follower_connection = 'udpin:127.0.0.1:14571'

# udp_pipe_to_d2 = MAVConnection(follower_connection, source_system=1)
#vehicle_master._handler.pipe(udp_pipe_to_d2)
#udp_pipe_to_d2.master.mav.srcComponent = 1
#udp_pipe_to_d2.start() # start the pipe to DroneKit_2

#-- Setup the commanded flying speed
gnd_speed = 0.5 # [m/s]

#-- Setup decelerating distance
dec_dist = 5 # [m]

#-- Setup tolerating distance
tol_dist = 0.075 # [m]

#-- Setup changing altitude speed
vertical_speed = 0.3 # [m/s]

#-- Setup decelerating altitude
vertical_dec = 0.5 # [m]

#-- Setup tolerating altitude
vertical_tol = 0.05 # [m]

#-- Define arm and takeoff. Still need to try whether use GPS or not. ONLY RUN WHEN INITIALIZE
def arm_and_takeoff(altitude_top, altitude_bot):

    if (using_sitl):
        # Check armable. Kata ka Zeke ga usah
        while not vehicle_master.is_armable:
            print("Waiting for master to be armable")
            time.sleep(1)

        vehicle_master.armed = True
        while not vehicle_master.armed: time.sleep(1)

        if (with_follower):
            while not vehicle_slave.is_armable:
                print("Waiting for slave to be armable")
                time.sleep(1)

            vehicle_slave.armed = True
            while not vehicle_slave.armed: time.sleep(1)

        print("Set mode to GUIDED")
        vehicle_master.mode = VehicleMode("GUIDED")

        if (with_follower):
            vehicle_slave.mode = VehicleMode("GUIDED")

        # Taking off
        print("Taking Off")
        vehicle_master.simple_takeoff(altitude_top)

        if (with_follower):
            vehicle_slave.simple_takeoff(altitude_bot)

        while True:
            v_alt = vehicle_master.location.global_relative_frame.alt

            if (with_follower):
                v_alt_slave = vehicle_slave.location.global_relative_frame.alt

                print(">> Altitude: Master = %.1f m. Slave = %.1f m."%(v_alt, v_alt_slave))

                if (v_alt >= altitude_top - 0.3 and v_alt_slave >= altitude_bot - 0.3):
                    print("Target altitude reached")
                    break

                time.sleep(1)
            else:
                print(">> Altitude: Master = %.1f m."%(v_alt))

                if (v_alt >= altitude_top - 0.3):
                    print("Target altitude reached")
                    break

                time.sleep(1)
    else:
        # Check armable. Kata ka Zeke ga usah
        while not vehicle_master.is_armable:
            print("Waiting for master to be armable")
            time.sleep(1)

        vehicle_master.armed = True
        while not vehicle_master.armed: time.sleep(1)

        if (with_follower):
            while not vehicle_slave.is_armable:
                print("Waiting for slave to be armable")
                time.sleep(1)

            vehicle_slave.armed = True
            while not vehicle_slave.armed: time.sleep(1)

        print("Set mode to GUIDED")
        vehicle_master.mode = VehicleMode("GUIDED")

        if (with_follower):
            vehicle_slave.mode = VehicleMode("GUIDED")

        # Taking off
        print("Taking Off")
        vehicle_master.simple_takeoff(altitude_top)

        if (with_follower):
            vehicle_slave.simple_takeoff(altitude_bot)

        while True:
            v_alt = vehicle_master.rangefinder.distance

            if (with_follower):
                v_alt_slave = vehicle_slave.rangefinder.distance

                print(">> Altitude: Master = %.1f m. Slave = %.1f m."%(v_alt, v_alt_slave))

                if (v_alt >= altitude_top - 0.5 and v_alt_slave >= altitude_bot - 0.5):
                    print("Target altitude reached")

                    print("Set mode to LOITER")
                    vehicle_master.mode = VehicleMode("LOITER")
                    vehicle_slave.mode = VehicleMode("LOITER")

                    break

                time.sleep(1)
            else:
                print(">> Altitude: Master = %.1f m."%(v_alt))

                if (v_alt >= altitude_top - 0.5):
                    print("Target altitude reached")

                    print("Set mode to LOITER")
                    vehicle_master.mode = VehicleMode("LOITER")

                    break

                time.sleep(1)

 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(target_vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    if (vz != 0):
        msg = target_vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111, #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                vx, vy, vz,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, 0)
    else:
        msg = target_vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111101000111, #-- BITMASK -> Consider only the velocities
                0, 0, 0,        #-- POSITION
                vx, vy, 0,     #-- VELOCITY
                0, 0, 0,        #-- ACCELERATIONS
                0, 0)
        
    target_vehicle.send_mavlink(msg)
    target_vehicle.flush()

#-- Define the function for sending mavlink yaw command in body frame
def condition_yaw(target_vehicle, heading, relative=True):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = target_vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    target_vehicle.send_mavlink(msg)

#-- Get velicity for changing altitude
def get_velocity_altitude(difference_altitude):
    """
    Get velocity for changing altitude. NEED TO BE REWORKED, TOO MUCH OVERSHOOTING
    """
    if (difference_altitude != 0):
        if (abs(difference_altitude) >= vertical_dec):
            return vertical_speed * difference_altitude / abs(difference_altitude)
        else:
            return vertical_speed * ((abs(difference_altitude) - vertical_tol) / vertical_dec) * difference_altitude / abs(difference_altitude)
    else:
        return 0

#-- Change altitude by velocity vector
def change_altitude(master_target_altitude, slave_target_altitude):
    """
    Change altitude with velocity vector and LocationGlobalRelative
    """
    print(">> Changing altitude")

    #-- Get current altitude
    if (using_sitl):
        v_alt = vehicle_master.location.global_relative_frame.alt
    else:
        v_alt = vehicle_master.rangefinder.distance
    
    if (with_follower):
        if (using_sitl):
            v_alt_slave = vehicle_slave.location.global_relative_frame.alt
        else:
            v_alt_slave = vehicle_slave.rangefinder.distance

    reached = False

    #-- Set the vehicle velocity
    while(not reached):
        if (not with_follower):
            print(">> Altitude: Master = %.2f m"%(v_alt))
            print(">> Target altitude: Master = %.2f m"%(master_target_altitude))

            master_difference = v_alt - master_target_altitude

            if (abs(master_difference) <= vertical_tol):
                reached = True

            master_speed = get_velocity_altitude(master_difference)

            print(">> Master speed = %.2f m/s"%(master_speed))
            set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)

            # v_alt = vehicle_master.rangefinder.distance
            v_alt = vehicle_master.location.global_relative_frame.alt

            time.sleep(0.1)
        else:
            print(">> Altitude: Master = %.2f m. Slave = %.2f"%(v_alt, v_alt_slave))
            print(">> Target altitude: Master = %.2f m. Slave = %.2f"%(master_target_altitude, slave_target_altitude))

            master_difference = v_alt - master_target_altitude
            slave_difference = v_alt_slave - slave_target_altitude

            if (abs(master_difference) <= vertical_tol and abs(slave_difference) <= vertical_tol):
                reached = True

            master_speed = get_velocity_altitude(master_difference)
            slave_speed = get_velocity_altitude(slave_difference)

            print(">> Master speed = %.2f m/s. Slave speed = %.2f m/s"%(master_speed, slave_speed))
            set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)
            set_velocity_body(vehicle_slave, vx=0, vy=0, vz=slave_speed)

            if (using_sitl):
                v_alt = vehicle_master.location.global_relative_frame.alt
                v_alt_slave = vehicle_slave.location.global_relative_frame.alt
            else:
                v_alt = vehicle_master.rangefinder.distance
                v_alt_slave = vehicle_slave.rangefinder.distance


            time.sleep(0.1)


def change_relative_altitude(master_target_altitude, slave_target_altitude):
    """
    Change relative altitude with velocity vector and LocationGlobalRelative
    master_target_altitude and slave_target_altitude are relative to the current altitude
    possitive = lebih tinggi, negative = lebih rendah
    """
    print(">> Changing relative altitude")

    #-- Get current altitude
    if (using_sitl):
        v_alt_init = vehicle_master.location.global_relative_frame.alt
    else:
        v_alt_init = vehicle_master.rangefinder.distance
    
    v_alt_rel = 0
    v_alt = v_alt_init
    
    if (with_follower):
        if (using_sitl):
            v_alt_slave_init = vehicle_slave.location.global_relative_frame.alt
        else:
            v_alt_slave_init = vehicle_slave.rangefinder.distance
    
        v_alt_slave_rel = 0
        v_alt_slave = v_alt_slave_init

    reached = False

    #-- Set the vehicle velocity
    while(not reached):
        if (not with_follower):
            print(">> Altitude: Master = %.2f m"%(v_alt_init + v_alt_rel))
            print(">> Target altitude: Master = %.2f m"%(v_alt_init + master_target_altitude))

            master_difference = v_alt_rel - master_target_altitude

            if (abs(master_difference) <= vertical_tol):
                reached = True

            master_speed = get_velocity_altitude(master_difference)

            print(">> Master speed = %.2f m/s"%(master_speed))
            set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)

            if (using_sitl):
                v_alt_rel = vehicle_master.location.global_relative_frame.alt - v_alt_init
            else:
                v_alt_rel = vehicle_master.rangefinder.distance - v_alt_init

            time.sleep(0.1)
        else:
            print(">> Altitude: Master = %.2f m. Slave = %.2f"%(v_alt_init + v_alt_rel, v_alt_slave_init + v_alt_slave_rel))
            print(">> Target altitude: Master = %.2f m. Slave = %.2f"%(master_target_altitude, slave_target_altitude))

            master_difference = v_alt_rel - master_target_altitude
            slave_difference = v_alt_slave_rel - slave_target_altitude

            if (abs(master_difference) <= vertical_tol and abs(slave_difference) <= vertical_tol):
                reached = True

            master_speed = get_velocity_altitude(master_difference)
            slave_speed = get_velocity_altitude(slave_difference)

            print(">> Master speed = %.2f m/s. Slave speed = %.2f m/s"%(master_speed, slave_speed))
            set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)
            set_velocity_body(vehicle_slave, vx=0, vy=0, vz=slave_speed)

            if (using_sitl):
                v_alt_rel = vehicle_master.location.global_relative_frame.alt
                v_alt_slave_rel = vehicle_slave.location.global_relative_frame.alt
            else:
                v_alt_rel = vehicle_master.rangefinder.distance - v_alt_init
                v_alt_slave_rel = vehicle_slave.rangefinder.distance - v_alt_init

            time.sleep(0.1)


#-- Get velocity for x-y plane
def get_speed(current_pos):
    """
    Get needed speed based on current distance to the target aruco marker

    Here, we have the decelerating distance (distance where the drone have to start decelerating), and we use 
    quadratic function to project the drone velocity based on the distance

    NEED TO BE REWORKED, TOO MUCH OVERSHOOTING
    """
    # Check if current position is not less than decelerating distance
    if (current_pos != 0):
        if (abs(current_pos) >= dec_dist):
            # Give velocity of maximum ground speed
            return gnd_speed * current_pos / abs(current_pos)
        else:
            # Use the quadratic function
            return (gnd_speed - (gnd_speed / dec_dist**2) * (abs(current_pos) - dec_dist)**2) * current_pos / abs(current_pos)
    else:
        return 0

#-- Move drone based on distance to nearest aruco marker
def gerakDrone(x, y):
    """
    Give the drone velocity vector. This function should be called every new x and y value is inputted from aruco detector
    """
    #-- Yaw safety
    condition_yaw(vehicle_master, heading=0)

    if (with_follower):
        condition_yaw(vehicle_slave, heading=0)

    print(f"Target Position: ({x}, {y})")

    # Set vx
    print(f"vx = {get_speed(x)}")
    set_velocity_body(vehicle_master, 0, get_speed(x), 0)
    if (with_follower):
        set_velocity_body(vehicle_slave, 0, get_speed(x), 0)

    # Check if has reached tolerating distance for drone to move in y direction
    if(abs(x) < tol_dist):
        # Set vy
        print(f"vy = {get_speed(y)}")
        set_velocity_body(vehicle_master, get_speed(y), 0, 0)
        if (with_follower):
            set_velocity_body(vehicle_slave, get_speed(y), 0, 0)

def gerakDroneEmergency(string):
    """
    Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
    """
    #-- Yaw safety
    condition_yaw(vehicle_master, 0)
    if (with_follower):
        condition_yaw(vehicle_slave, 0)

    if string == "RIGHT":
        set_velocity_body(vehicle_master, 0, gnd_speed, 0)
        if (with_follower):
            set_velocity_body(vehicle_slave, 0, gnd_speed, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle_master, 0, -gnd_speed, 0)
        if (with_follower):
            set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "FORWARD":
        set_velocity_body(vehicle_master, gnd_speed, 0, 0)
        if (with_follower):
            set_velocity_body(vehicle_slave, gnd_speed, 0, 0)
    elif string == "BACKWARD":
        set_velocity_body(vehicle_master, -gnd_speed, 0, 0)
        if (with_follower):
            set_velocity_body(vehicle_slave, -gnd_speed, 0, 0)

def landDrone():
    vehicle_master.mode = VehicleMode("LAND")

    if (with_follower):
        vehicle_slave.mode = VehicleMode("LAND")

#-- detect wifi conenction (not internet) using socket
# wifi_ip = "192.168.0.102"
wifi_ip = "192.168.0.100"
# wifi_ip = "192.168.0.1"

def is_wifi():
    """
    Query internet using python
    :return:
    """
    try:
        # create a socket to an address on the local network
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((wifi_ip, 80)) # change the IP address to your router's IP
        return True
    except:
        return False

#Wifi link cutoff emergency stop
def wifi_stop():
    """
    Stop the drone (or rather the motor of drone) when wifi is disconnected
    """

    # Check wifi connection
    while is_wifi():
        continue
        # print("Connected")
    
    print("Disconnected")

    # Disarming the vehicle
    print("Disarming the vehicle")
    vehicle_master.armed = False

    sys.exit(1)

#-- Main function
if __name__ == '__main__':
    check_wifi = threading.Thread(target=wifi_stop)
    check_wifi.start()
    
    arm_and_takeoff(2, 1.5)

    for i in range(100, 0, -2):
        print(f"x, y = {i**2 / 10000}, {i**2 / 40000}")

        gerakDrone(i**2 / 10000, i**2  / 40000)

        time.sleep(0.1)

    for i in range(100, 0, -2):
        print(f"x, y = 0, {i**2 / 400}")

        gerakDrone(0, i**2  / 400)

        time.sleep(0.1)

    for i in range(100, 0, -2):
        print(f"x, y = {i**2 / 10000}, {10 - i**2 / 10000}")

        gerakDrone(i**2 / 10000, 10 - i**2 / 10000)

        time.sleep(0.1)
    
    for i in range(100, 0, -2):
        print(f"x, y = {0 * i**2 / 10000}, {10 - i**2 / 10000}")

        gerakDrone(0 * i**2 / 10000, 10 - i**2 / 10000)

        time.sleep(0.1)

    # landDrone()