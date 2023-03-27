# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np

from cv2 import aruco

commandDict = {
  '0': 'FORWARD',
  '1': 'LEFT',
  '2': 'FORWARD',
  '3': 'FORWARD',
  '4': 'FORWARD',
  '5': 'FORWARD',
  '6': 'FORWARD',
  '7': 'FORWARD',
  '8': 'FORWARD',
  '9': 'FORWARD',
  '10': 'FORWARD',
  '11' : 'RIGHT',
  '12' : 'RIGHT',
  '13' : 'RIGHT',
  '14' : 'RIGHT',
  '15' : 'RIGHT',
  '16' : 'RIGHT',
  '17' : 'RIGHT',
  '18' : 'RIGHT',
  '19' : 'RIGHT',
  '20' : 'RIGHT',
  '21' : 'LEFT',
  '22' : 'LEFT',
  '23' : 'LEFT',
  '24' : 'LEFT',
  '25' : 'LEFT',
  '26' : 'LEFT',
  '27' : 'LEFT',
  '28' : 'LEFT',
  '29' : 'LEFT',
  '30' : 'LEFT',
  '31' : 'BACKWARD',
  '32' : 'BACKWARD',
  '33' : 'BACKWARD',
  '34' : 'BACKWARD',
  '35' : 'BACKWARD',
  '36' : 'BACKWARD',
  '37' : 'BACKWARD',
  '38' : 'BACKWARD',
  '39' : 'BACKWARD',
  '40' : 'BACKWARD',
    
    #   specCommandDict
  '100': 'LEFT', 
  '101': 'FORWARD',
  '102': 'RIGHT',
  '103': 'BACKWARD',
}

"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.
The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:
https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                     [            0, intrinsics.fy, intrinsics.ppy],
                     [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics): # Sama saja dengan distortion coefficient
    return np.array(intrinsics.coeffs[:4])

# Set up a mutex to share data between threads 
from threading import Lock
frame_mutex = Lock()
frame_data = {"left"  : None,
              "right" : None,
              "timestamp_ms" : None
              }

"""
This callback is called on a separate thread, so we must use a mutex
to ensure that data is synchronized properly. We should also be
careful not to do much work on this thread to avoid data backing up in the
callback queue.
"""
def callback(frame):
    global frame_data
    if frame.is_frameset():
        frameset = frame.as_frameset()
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        frame_mutex.acquire()
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["timestamp_ms"] = ts
        frame_mutex.release()

def poseEstim(currId, bannedId, IdDistArr, streams, isNewId):

    # Getting intrinsics from streams
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                "right" : streams["right"].get_intrinsics()}

    # Translate the intrinsics from librealsense into OpenCV
    # We only use the left camera!!
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])

    MARKER_SIZE = 28  # centimeters
    marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_50) # Aruco type
    param_markers = aruco.DetectorParameters_create()

    while True:
        # Check if the camera has acquired any frames
        frame_mutex.acquire()
        valid = frame_data["timestamp_ms"] is not None
        frame_mutex.release()

        # If frames are ready to process
        if valid:
            # Hold the mutex only long enough to copy the stereo frames
            frame_mutex.acquire()
            frame_copy = {"left"  : frame_data["left"].copy(),
                        "right" : frame_data["right"].copy()}
            frame_mutex.release()

            marker_corners, marker_IDs, reject = aruco.detectMarkers(
                frame_copy["left"], marker_dict, parameters=param_markers
            )
            
            if marker_corners:
                # Getting tVec from estimatePoseSingleMarkers 
                _, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, K_left, D_left
                )
                total_markers = range(0, marker_IDs.size)
                for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                    cv2.polylines(
                        frame_copy["left"], [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                    )

                    distance = np.sqrt(
                        tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                    )
                    
                    # if want to delete the key, just type del idDistArr[ids[i]]        

                    # Draw the pose of the marker
                    x = round(tVec[i][0][0],1)
                    y = round(tVec[i][0][1],1)

                    # Retrieving the command from command dictionary
                    command = commandDict.get(f"{ids[0]}")
                    
                    if (ids[0] not in bannedId) :
                        # Updating distance in IdDistArr if id has not been used before
                        IdDistArr[ids[0]] = distance
                        
                        if (isNewId):
                            # If looking for new marker, then quickly return the found id 
                            return (x,y,IdDistArr,ids[0],command)

                    if (ids[0] == currId and not isNewId):
                        # If id is the current target id and not searching for new marker
                        print(f"id: {ids[0]} x:{x} y: {y} command: {command}\n")
                        return (x,y,IdDistArr,ids[0],command) 

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

if __name__ == "__main__":
    currId = 1
    bannedId=[]
    poseEstim(currId, bannedId)