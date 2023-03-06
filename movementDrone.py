"""
Script to move drone based on aruco
"""

#-- LIBRARY IMPORTS --#

import time
import math

# DroneKit
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

# WiFi
import socket
import threading

# Force Quit
import sys

#-- Connect to the vehicle
print('Connecting Master Drone...')
vehicle_master = connect('udp:127.0.0.1:14571')

print('Connecting Slave Drone...')
vehicle_slave = connect('udp:127.0.0.1:14561')

#-- Setup the commanded flying speed
gnd_speed = 0.5 # [m/s]

#-- Setup decelerating distance
dec_dist = 2 # [m]

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

   # Check armable. Kata ka Zeke ga usah
    while not vehicle_master.is_armable:
        print("Waiting for master to be armable")
        time.sleep(1)

    while not vehicle_slave.is_armable:
        print("Waiting for slave to be armable")
        time.sleep(1)

    print("Arming master motors")
    vehicle_master.mode = VehicleMode("GUIDED")
    vehicle_master.armed = True

    while not vehicle_master.armed: time.sleep(1)

    print("Arming slave motors")
    vehicle_slave.mode = VehicleMode("GUIDED")
    vehicle_slave.armed = True

    while not vehicle_slave.armed: time.sleep(1)

    # Taking off
    print("Taking Off")
    vehicle_master.simple_takeoff(altitude_top)
    vehicle_slave.simple_takeoff(altitude_bot)

    while True:
        v_alt = vehicle_master.location.global_relative_frame.alt
        v_alt_slave = vehicle_slave.location.global_relative_frame.alt
        print(">> Altitude: Master = %.1f m, Slave = %.1f m"%(v_alt, v_alt_slave))
        if (v_alt >= altitude_top - 0.3 and v_alt_slave >= altitude_bot - 0.3):
            print("Target altitude reached")
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
    msg = target_vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
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
    Get velocity for changing altitude
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
    v_alt = vehicle_master.location.global_relative_frame.alt
    v_alt_slave = vehicle_slave.location.global_relative_frame.alt

    reached = False

    #-- Set the vehicle velocity
    while(not reached):
        print(">> Altitude: Master = %.2f m, Slave = %.2f m"%(v_alt, v_alt_slave))
        print(">> Target altitude: Master = %.2f m, Slave = %.2f m"%(master_target_altitude, slave_target_altitude))

        master_difference = v_alt - master_target_altitude
        slave_difference = v_alt_slave - slave_target_altitude

        if (abs(master_difference) <= vertical_tol and abs(slave_difference) <= vertical_tol):
            reached = True

        master_speed = get_velocity_altitude(master_difference)

        print(">> Master speed = %.2f m/s"%(master_speed))
        set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)

        slave_speed = get_velocity_altitude(slave_difference)

        print(">> Slave speed = %.2f m/s"%(slave_speed))
        set_velocity_body(vehicle_slave, vx=0, vy=0, vz=slave_speed)

        v_alt = vehicle_master.location.global_relative_frame.alt
        v_alt_slave = vehicle_slave.location.global_relative_frame.alt

        time.sleep(0.1)

#-- Change altitude with simple_goto
def change_altitude_simplegoto(master_target_altitude, slave_target_altitude):
    """
    Change altitude with simple_goto
    """
    print(">> Changing altitude")

    #-- Another constant
    change_vertical_tol = 0.08

    #-- Get current altitude
    v_alt = vehicle_master.location.global_relative_frame.alt
    v_alt_slave = vehicle_slave.location.global_relative_frame.alt

    reached = False

    #-- Set the vehicle velocity
    while(not reached):
        print(">> Altitude: Master = %.2f m, Slave = %.2f m"%(v_alt, v_alt_slave))
        print(">> Target altitude: Master = %.2f m, Slave = %.2f m"%(master_target_altitude, slave_target_altitude))

        master_difference = v_alt - master_target_altitude
        slave_difference = v_alt_slave - slave_target_altitude

        if (abs(master_difference) <= change_vertical_tol and abs(slave_difference) <= change_vertical_tol):
            reached = True

        master_location = LocationGlobalRelative(vehicle_master.location.global_relative_frame.lat, vehicle_master.location.global_relative_frame.lon, master_target_altitude)
        vehicle_master.simple_goto(master_location)

        slave_location = LocationGlobalRelative(vehicle_slave.location.global_relative_frame.lat, vehicle_slave.location.global_relative_frame.lon, slave_target_altitude)
        vehicle_slave.simple_goto(slave_location)

        v_alt = vehicle_master.location.global_relative_frame.alt
        v_alt_slave = vehicle_slave.location.global_relative_frame.alt

        time.sleep(0.1)


#-- Get velocity for x-y plane
def get_speed(current_pos):
    """
    Get needed speed based on current distance to the target aruco marker

    Here, we have the decelerating distance (distance where the drone have to start decelerating), and we use 
    quadratic function to project the drone velocity based on the distance
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
    condition_yaw(vehicle_slave, heading=0)

    print(f"Target Position: ({x}, {y})")

    # Set vx
    print(f"vx = {get_speed(x)}")
    set_velocity_body(vehicle_master, 0, get_speed(x), 0)
    set_velocity_body(vehicle_slave, 0, get_speed(x), 0)

    # Check if has reached tolerating distance for drone to move in y direction
    if(abs(x) < tol_dist):
        # Set vy
        print(f"vy = {get_speed(y)}")
        set_velocity_body(vehicle_master, get_speed(y), 0, 0)
        set_velocity_body(vehicle_slave, get_speed(y), 0, 0)

def gerakDroneEmergency(string):
    """
    Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
    """
    #-- Yaw safety
    condition_yaw(vehicle_master, 0)
    condition_yaw(vehicle_slave, 0)

    if string == "RIGHT":
        set_velocity_body(vehicle_master, 0, gnd_speed, 0)
        set_velocity_body(vehicle_slave, 0, gnd_speed, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle_master, 0, -gnd_speed, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "FORWARD":
        set_velocity_body(vehicle_master, gnd_speed, 0, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "BACKWARD":
        set_velocity_body(vehicle_master, -gnd_speed, 0, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)

def landDrone():
    vehicle_master.mode = VehicleMode("LAND")
    vehicle_slave.mode = VehicleMode("LAND")

#-- detect wifi conenction (not internet) using socket
wifi_ip = "192.168.88.61"

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
    vehicle_slave.armed = False

    sys.exit(1)

if __name__ == "__main__":
    #---- MAIN FUNCTION
    #- Check wifi thread
    check_wifi = threading.Thread(target=wifi_stop)
    check_wifi.start()

    #- Takeoff
    try: 
        arm_and_takeoff(2, 1.5)

        set_velocity_body(vehicle_slave, get_speed(5), 0, 0)

        time.sleep(2)

        set_velocity_body(vehicle_slave, 0, 0, 0)

        time.sleep(10)

        i = 4
        j = 4
        k = 0

        change_altitude(0.5, 0.5)

        while (i >= -0.000005 and j >= -0.000005):
            print(f"i = {round(i, 2)}, j = {round(j, 2)}")

            gerakDrone(round(i, 2), round(j, 2))

            if (i > 0):
                i = round(i, 2) - 0.05
            else:
                if (j > 0):
                    print("move y")
                    j = round(j, 2) - 0.05
            
            if (round(i, 2) == 0 and round(j, 2) == 0):
                for k in range (0, 10):
                    gerakDrone(0, 0)
                    time.sleep(0.1)

                print("Kelar")

                break
            
            time.sleep(0.1)

        i = -2
        j = -5
        k = 0

        while (i <= 0.000005 and j <= -0.000005):
            print(f"i = {round(i, 2)}, j = {round(j, 2)}")

            gerakDrone(round(i, 2), round(j, 2))

            if (i < 0):
                i = round(i, 2) + 0.05
            else:
                if (j < 0):
                    print("move y")
                    j = round(j, 2) + 0.05
            
            if (round(i, 2) == 0 and round(j, 2) == 0):
                for k in range (0, 10):
                    gerakDrone(0, 0)
                    time.sleep(0.1)

                print("Kelar")

                break
            
            time.sleep(0.1)
        
        change_altitude(2, 1.5)

        i = 2
        j = -4
        k = 0

        while (i >= 0.000005 and j <= -0.000005):
            print(f"i = {round(i, 2)}, j = {round(j, 2)}")

            gerakDrone(round(i, 2), round(j, 2))

            if (i > 0):
                i = round(i, 2) - 0.05
            else:
                if (j < 0):
                    print("move y")
                    j = round(j, 2) + 0.05
            
            if (round(i, 2) == 0 and round(j, 2) == 0):
                for k in range (0, 10):
                    gerakDrone(0, 0)
                    time.sleep(0.1)

                print("Kelar")

                break
            
            time.sleep(0.1)

        change_altitude(0, 0)
        landDrone()
    finally:
        landDrone()

    print("Finished")

    check_wifi.join()
