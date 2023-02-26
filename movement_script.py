"""
Script to move drone based on aruco
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')

#-- Setup the commanded flying speed
gnd_speed = 0.5 # [m/s]

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
# Get speed for decelaring
def get_speed(current_pos):
    if (current_pos > 0):
        # for positive x value (right)
        return -0.125(current_pos-2)^2 + 0.5
    else:
        # for negative x value (left)
        return 0.125(current_pos-2)^2 - 0.5

# Give the drone velocity vector. This function should be called every new x and y value is inputted from aruco detector
def gerakDrone(x, y):
    # First, we check the current x position relative to the aruco and move in the x axis based on that position
    # Here, we set x = 2 m as decelerating threshold
    if (abs(x) >= 2):
        set_velocity_body(vehicle, gnd_speed * x / abs(x), 0, 0)
    else:
        set_velocity_body(vehicle, get_speed(x), 0, 0)

        # Here, we set x = 0.1 as x position tolerance before the drone starts moving in y direction
        if(abs(x) > 0.1):
            if(abs(y) >= 2):
                set_velocity_body(vehicle, 0, gnd_speed * y / abs(y), 0)
            else:
                set_velocity_body(vehicle, 0, get_speed(y), 0)

# Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
def gerakDroneEmergency(string):
    if string == "FORWARD":
        set_velocity_body(vehicle, 0, gnd_speed, 0)
    elif string == "BACKWARD":
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif string == "RIGHT":
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    
#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)