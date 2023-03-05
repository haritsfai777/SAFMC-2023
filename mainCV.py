# First import the library
from disEst import poseEstim, callback
import pyrealsense2 as rs
import numpy as np

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
vehicle_master = connect('/dev/ttyUSB0', baud=57600)

# print('Connecting Slave Drone...')
# vehicle_slave = connect('udp:127.0.0.1:14561')

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
def arm_and_takeoff(altitude_top, altitude_bot, pipe):

   # Check armable. Kata ka Zeke ga usah
    # while not vehicle_master.is_armable:
    #     print("Waiting for master to be armable")
    #     time.sleep(1)

    # while not vehicle_slave.is_armable:
    #     print("Waiting for slave to be armable")
    #     time.sleep(1)

    # print("Arming master motors")
    vehicle_master.mode = VehicleMode("GUIDED")
    # vehicle_master.armed = True

    # while not vehicle_master.armed: time.sleep(1)

    # print("Arming slave motors")
    # vehicle_slave.mode = VehicleMode("GUIDED")
    # vehicle_slave.armed = True

    # while not vehicle_slave.armed: time.sleep(1)

    # Taking off
    print("Taking Off")
    vehicle_master.simple_takeoff(altitude_top)
    # vehicle_slave.simple_takeoff(altitude_bot)

    while True:
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()

        pose_data = pose.get_pose_data()

        # v_alt = vehicle_master.location.global_relative_frame.alt
        v_alt = pose_data.translation.z

        # v_alt_slave = vehicle_slave.location.global_relative_frame.alt
        print(">> Altitude: Master = %.1f m"%(v_alt))
        if (v_alt >= altitude_top - 0.3):
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
def change_altitude(master_target_altitude, slave_target_altitude, pose):
    """
    Change altitude with velocity vector and LocationGlobalRelative
    """
    print(">> Changing altitude")

    #-- Get current altitude
    pose_data = pose.get_pose_data()

    v_alt = pose_data.translation.z
    # v_alt_slave = vehicle_slave.location.global_relative_frame.alt

    reached = False

    #-- Set the vehicle velocity
    while(not reached):
        print(">> Altitude: Master = %.2f m"%(v_alt))
        print(">> Target altitude: Master = %.2f m"%(master_target_altitude))

        master_difference = v_alt - master_target_altitude
        # slave_difference = v_alt_slave - slave_target_altitude

        if (abs(master_difference) <= vertical_tol):
            reached = True

        master_speed = get_velocity_altitude(master_difference)

        print(">> Master speed = %.2f m/s"%(master_speed))
        set_velocity_body(vehicle_master, vx=0, vy=0, vz=master_speed)

        # slave_speed = get_velocity_altitude(slave_difference)

        # print(">> Slave speed = %.2f m/s"%(slave_speed))
        # set_velocity_body(vehicle_slave, vx=0, vy=0, vz=slave_speed)

        # v_alt = vehicle_master.location.global_relative_frame.alt
        # v_alt_slave = vehicle_slave.location.global_relative_frame.alt

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
def gerakDrone(x, y, pose):
    """
    Give the drone velocity vector. This function should be called every new x and y value is inputted from aruco detector
    """
    #-- Yaw safety
    condition_yaw(vehicle_master, heading=0)
    # condition_yaw(vehicle_slave, heading=0)

    print(f"Target Position: ({x}, {y})")

    # Set vx
    print(f"vx = {get_speed(x)}")
    set_velocity_body(vehicle_master, 0, get_speed(x), 0)
    # set_velocity_body(vehicle_slave, 0, get_speed(x), 0)

    # Check if has reached tolerating distance for drone to move in y direction
    if(abs(x) < tol_dist):
        # Set vy
        print(f"vy = {get_speed(y)}")
        set_velocity_body(vehicle_master, get_speed(y), 0, 0)
        # set_velocity_body(vehicle_slave, get_speed(y), 0, 0)

def gerakDroneEmergency(string):
    """
    Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
    """
    #-- Yaw safety
    condition_yaw(vehicle_master, 0)
    # condition_yaw(vehicle_slave, 0)

    if string == "RIGHT":
        set_velocity_body(vehicle_master, 0, gnd_speed, 0)
        # set_velocity_body(vehicle_slave, 0, gnd_speed, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle_master, 0, -gnd_speed, 0)
        # set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "FORWARD":
        set_velocity_body(vehicle_master, gnd_speed, 0, 0)
        # set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)
    elif string == "BACKWARD":
        set_velocity_body(vehicle_master, -gnd_speed, 0, 0)
        # set_velocity_body(vehicle_slave, 0, -gnd_speed, 0)

def landDrone():
    vehicle_master.mode = VehicleMode("LAND")
    # vehicle_slave.mode = VehicleMode("LAND")

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
    # vehicle_slave.armed = False

    sys.exit(1)


# from dronewifi import *

def threshold(x,y):
	if (abs(x) <= 25 and abs(y) <= 25):
		return True

def changeId(idDistArr, currId):
  # sort tuple idDistArr by distance idDistArr = {id:distanc, id:distance, ...}
  sorted_dict = sorted(idDistArr.items(), key=lambda x: x[1])

  # get the id of the seconde closest marker
  if len(sorted_dict) >= 2:
    id = sorted_dict[1][0]
    return id

  elif (len(sorted_dict) == 1 and sorted_dict[0][0] != currId):
    return sorted_dict[0][0]

  else:
    return None

def arucoFollower(): 
    currId = 0

    # Untuk testing, jangan lupa diganti dengan yang di bawah
    # lastId = 0
    lastId = 5

    bannedId = []

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and stream everything
    cfg = rs.config()

    # Start streaming with our callback
    pipe.start(cfg, callback)

    arm_and_takeoff(2, 1.5, pipe)

    try:
      IdDistArr = {}

      # Retreive the stream and intrinsic properties for both cameras
      profiles = pipe.get_active_profile()
      streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
          "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}

      isNewId = False
      print("Mulai mendeteksi markers\n")
      x, y, idDistArr, id, command = poseEstim(currId, bannedId, IdDistArr, streams, isNewId)

      # First loop until the last normal id
      while (currId != lastId) or (currId == lastId and not threshold(x, y)):
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()

        # Cek threshold
        if threshold(x,y):
          bannedId.append(currId)
          print(f"banned id: {bannedId}")
          newId = changeId(idDistArr, currId)

          # Melakukan loop hingga Id baru tidak None 
          while newId is None:
            print("Scanning other aruco markers")
            isNewId = True
            x, y, idDistArr, id, command = poseEstim(currId, bannedId, IdDistArr, streams, isNewId)
            newId = changeId(idDistArr, currId)
            isNewId = False

          # Menghapus id dan distance dari aruco yang telah dilewati dari array idDistArr
          del idDistArr[currId]

          # Memasukkan id target baru
          currId = newId

        x, y, idDistArr, id, command = poseEstim(currId, bannedId, IdDistArr, streams, isNewId)
        gerakDrone(x * 0.01, -y * 0.01, pose)

      print("Selesai rute awal, memulai rute penurunan payload")
      print("Switching to final markers")
      # Second loop from special dictionary command last special id
      # Loop the id with increment

      # untuk testing jangan lupa nanti diganti dengan yang dicomment di bawah
      currId = 47 # First id of special command 
      lastIdSp = 49 # Last id of special command

      # currId = 46 # First id of special command 
      # lastIdSp = 49 # Last id of special command
 
      change_altitude(2.5, 2, pose) # Naik untuk persiapan pelepasan payload

      while (command != "UP"):
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()

        print("Now UP!")
        x, y, idDistArr, id, command = poseEstim(currId, bannedId, IdDistArr, streams, isNewId)
        gerakDrone(x * 0.01, -y * 0.01, pose)
        
        # Cek threshold
        if threshold(x,y):
          # Jika id yang sekarang bukan id terakhir
          if (currId != lastIdSp):
              print("Change next id")
              currId+=1
          
          # jika id yang sekarang adalah id terakhir, maka commandnya hanya UP
          else :
            change_altitude(2, 2.5, pose)
            print("Up")
            time.sleep(2)
            command = "UP"

    finally:
      print("Dropping drone...")
      landDrone() # landing jika sudah selesai ataupun jika keyboard interrupt
      print("Selesai")
      pipe.stop()

# Main function
if __name__ == "__main__":
  check_wifi = threading.Thread(target=wifi_stop)
  check_wifi.start()
  # arm_and_takeoff(2, 1.5)
  arucoFollower()
  check_wifi.join()