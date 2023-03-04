"""
Script to move drone based on aruco
"""


import time, math
import urllib
from urllib.request import urlopen
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import threading
import socket
import sys

#-- Connect to the vehicle
print('Connecting Master Drone...')
vehicle = connect('udp:127.0.0.1:14561')

print('Connecting Slave Drone...')
vehicle_slave = connect('udp:127.0.0.1:14571')

#-- Setup the commanded flying speed
gnd_speed = 0.5 # [m/s]

#-- Setup decelerating distance
dec_dist = 2 # [m]

#-- Setup tolerating distance
tol_dist = 0.075 # [m]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("Waiting for master to be armable")
      time.sleep(1)

   while not vehicle_slave.is_armable:
      print("Waiting for slave to be armable")
      time.sleep(1)

   print("Arming master motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Arming slave motors")
   vehicle_slave.mode = VehicleMode("GUIDED")
   vehicle_slave.armed = True

   while not vehicle_slave.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)
   vehicle_slave.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      v_alt_slave = vehicle_slave.location.global_relative_frame.alt
      print(">> Altitude: Master = %.1f m, Slave = %.1f m"%(v_alt, v_alt_slave))
      if (v_alt >= altitude - 0.2 and v_alt_slave >= altitude - 0.2):
          print("Target altitude reached")
          break
      time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

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

#-- Set yaw
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

def get_speed(current_pos):
    """
    Get speed based on current distance to the target aruco marker

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

def gerakDrone(x, y):
    """
    Give the drone velocity vector. This function should be called every new x and y value is inputted from aruco detector
    """
    condition_yaw(vehicle, heading=0)
    condition_yaw(vehicle_slave, heading=0)

    print(f"Target Position: ({x}, {y})")

    # Set vx
    print(f"vx = {get_speed(x)}")
    set_velocity_body(vehicle, 0, get_speed(x), 0)
    set_velocity_body(vehicle_slave, 0, get_speed(x), 0)

    # Check if has reached tolerating distance for drone to move in y direction
    if(abs(x) < tol_dist):
        # Set vy
        print(f"vy = {get_speed(y)}")
        set_velocity_body(vehicle, get_speed(y), 0, 0)
        set_velocity_body(vehicle_slave, get_speed(y), 0, 0)

def gerakDroneEmergency(string):
    """
    Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
    """
    condition_yaw(vehicle, 0)
    condition_yaw(vehicle_slave, 0)

    if string == "FORWARD":
        set_velocity_body(vehicle, 0, gnd_speed, 0)
        set_velocity_body(vehicle_slave, 0, gnd_speed, 0)
    elif string == "BACKWARD":
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "RIGHT":
        set_velocity_body(vehicle, gnd_speed, 0, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
        set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)

def printAltitude():
    for i in range(0,8):
        v_alt = vehicle.location.global_relative_frame.alt
        v_alt_slave = vehicle_slave.location.global_relative_frame.alt
        print(">> Altitude: Master = %.1f m, Slave = %.1f m"%(v_alt, v_alt_slave))

        time.sleep(0.5)

def modeDrop(altitude_top, altitude_bot):
    """
    Run this when "UP" command is received
    """
    print("Last Aruco Reached")

    print_altitude = threading.Thread(target=printAltitude)
    print_altitude.start()

    set_velocity_body(vehicle, 0, 0, 0.25)
    set_velocity_body(vehicle_slave, 0, 0, -0.25)

    time.sleep(2)

    set_velocity_body(vehicle, 0, 0, 0.1)
    set_velocity_body(vehicle_slave, 0, 0, -0.1)

    time.sleep(2)

    set_velocity_body(vehicle, 0, 0, 0)
    set_velocity_body(vehicle_slave, 0, 0, 0)

    print("Reached Altitude")

    # master_current_location = vehicle.location.global_relative_frame
    # slave_current_location = vehicle_slave.location.global_relative_frame

    # print("Taking Off...")
    # master_location = LocationGlobalRelative(master_current_location.lat, master_current_location.lon, altitude_top)
    # slave_location = LocationGlobalRelative(slave_current_location.lat, slave_current_location.lon, altitude_bot)

    # vehicle.simple_goto(master_location)
    # vehicle_slave.simple_goto(slave_location)

    # while True:
    #     v_alt = vehicle.location.global_relative_frame.alt
    #     v_alt_slave = vehicle_slave.location.global_relative_frame.alt
    #     print(">> Altitude: Master = %.1f m, Slave = %.1f m"%(v_alt, v_alt_slave))
    #     if (v_alt >= altitude_top - 1.0 and v_alt_slave <= altitude_bot + 1.0):
    #         print("Target altitude reached")
    #         break
    #     time.sleep(1)

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
    vehicle.armed = False
    vehicle_slave.armed = False

    sys.exit(1)

if __name__ == "__main__":
    #---- MAIN FUNCTION
    #- Check wifi thread
    check_wifi = threading.Thread(target=wifi_stop)
    check_wifi.start()

    #- Takeoff
    arm_and_takeoff(2)

    set_velocity_body(vehicle_slave, get_speed(5), 0, 0)

    time.sleep(2)

    set_velocity_body(vehicle_slave, 0, 0, 0)

    time.sleep(5)

    i = 4
    j = 4
    k = 0

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

    i = 2
    j = -4
    k = 0

    while (i >= 0.000005 and j <= -0.000005):
        print(f"i = {round(i, 2)}, j = {round(j, 2)}")

        gerakDrone(round(i, 2), round(j, 2))

        if (i < 0):
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

    modeDrop(1.5, 2.5)

    print("Finished")

    check_wifi.join()
