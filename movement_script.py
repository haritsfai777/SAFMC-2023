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
    
def get_speed(current_pos):
    # For decelerating speed
    if (current_pos > 0):
        #for positive x value (right)
        return -0.125(current_pos-2)^2 + 0.5
    else:
        #for negative x value (left)
        return -0.125(current_pos-2)^2 - 0.5

#-- Move from aruco
def gerakDrone(x, y):
    # Check x position relative to the next aruco
    if (x >= 2):
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif (x <= -2):
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    elif (x < 2 and x >= 0):
        #decelerates velocity down to 0
        set_velocity_body(vehicle, get_speed(x), 0, 0)
        #once velocity has reached 0
        if (-0.1 < x < 0.1):
            # After we reached the x threshold, stop x velocity and move y function
            if (y >= 2):
                set_velocity_body(vehicle, 0, gnd_speed, 0)
            elif (y <= -2):
                set_velocity_body(vehicle, 0, -gnd_speed, 0)
            elif (y < 2 and y >= 0):
                set_velocity_body(vehicle, 0, get_speed(y), 0)
            elif (y > -2 and y <= 0):
                set_velocity_body(vehicle, 0, get_speed(y), 0)
    elif (x > -2 and x <= 0):
        set_velocity_body(vehicle, get_speed(x), 0, 0)
        if (-0.1 < x < 0.1):
            if (y >= 2):
                set_velocity_body(vehicle, 0, gnd_speed, 0)
            elif (y <= -2):
                set_velocity_body(vehicle, 0, -gnd_speed, 0)
            elif (y < 2 and y >= 0):
                set_velocity_body(vehicle, 0, get_speed(y), 0)
            elif (y > -2 and y <= 0):
                set_velocity_body(vehicle, 0, get_speed(y), 0)

# # --Move drone based on input from aruco with PID
# setpoint    = 0

# error       = 0
# integral    = 0
# derivative  = 0

# error_prev  = 0

# dt          = 0.01                  # data time sampling

# # need tuning
# Kp          = 0.05                  # proportional coefficient
# Ki          = 0.0005                # integral coefficient
# Kd          = 0.0000000005          # derivative coefficient

# giliran_x_gerak = True

# def gerakDronePID(x, y):
#     global error_prev, integral, derivative, giliran_x_gerak

#     if (giliran_x_gerak):
#         if (abs(x) > 0):
#             set_velocity_body(vehicle, getSpeedPID(x), 0, 0)
#         else:
#             set_velocity_body(vehicle, 0, 0, 0)

#             error_prev = 0
#             integral = 0
#             derivative = 0

#             giliran_x_gerak = False
#     else:
#         if (abs(y) > 0):
#             set_velocity_body(vehicle, 0, getSpeedPID(y), 0)
#         else:
#             set_velocity_body(vehicle, 0, 0, 0)

#             error_prev = 0
#             integral = 0
#             derivative = 0

#             giliran_x_gerak = True 

# def getSpeedPID(feedback):
#     global error_prev, integral, derivative

#     # Get the error
#     error = feedback - setpoint
#     de = error - error_prev

#     # Summate the integral
#     integral = integral + (error + error_prev) * dt / 2           # trapezoidal rule

#     # Get the derivative
#     derivative = de / dt

#     # Get the speed
#     speed = Kp * error + Ki * integral + Kd * derivative

#     # Simpan previous error
#     error_prev = error

#     return max(speed, gnd_speed)

def gerakDroneEmergency(string):
    # Used only if x and y position is not detected, manually set the direction based on command from the previous aruco marker
    if string == "FORWARD":
        set_velocity_body(vehicle, 0, gnd_speed, 0)
    elif string == "BACK":
        set_velocity_body(vehicle, 0, -gnd_speed, 0)
    elif string == "RIGHT":
        set_velocity_body(vehicle, gnd_speed, 0, 0)
    elif string == "LEFT":
        set_velocity_body(vehicle, -gnd_speed, 0, 0)
    
    
#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)