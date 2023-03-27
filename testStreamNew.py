#####################################################
##          librealsense T265 streams test         ##
#####################################################
# This assumes .so file is found on the same directory
import pyrealsense2 as rs

# Prettier prints for reverse-engineering
from disCam import *
from pprint import pprint
import cv2
from cv2 import aruco
import numpy as np

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

# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('Found device:', devices[i].get_info(rs.camera_info.name), ', with serial number: ', devices[i].get_info(rs.camera_info.serial_number))

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))

# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)

try:
    # Set up an OpenCV window to visualize the results
    WINDOW_TITLE = 'Realsense'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)

    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
            "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                "right" : streams["right"].get_intrinsics()}

    # Print information about both cameras
    print("Left camera:",  intrinsics["left"])
    print("Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)


    # Get the relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(streams["left"], streams["right"])

    MARKER_SIZE = 17  # centimeters
    marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_100)
    param_markers = aruco.DetectorParameters_create()
    wait = True 

    while True:
        # print("p")

        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())
        # print(left_data)

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        # print('Left frame', left_data.shape)
        # print('Right frame', right_data.shape)

        # # Check if the camera has acquired any frames
        # frame_mutex.acquire()
        # valid = frame_data["timestamp_ms"] is not None
        # frame_mutex.release()

        # If frames are ready to process
        # if valid:
        # Hold the mutex only long enough to copy the stereo frames
        frame_mutex.acquire()
        frame_copy = {"left"  : left_data,
                    "right" : right_data}
        frame_mutex.release() 
        # frame_mutex.acquire()
        # frame_copy = {"left"  : frame_data["left"].copy(),
        #             "right" : frame_data["right"].copy()}
        # frame_mutex.release()
        # gray_frame = cv2.cvtColor(frame_copy["left"], cv2.COLOR_BGR2GRAY)

        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            frame_copy["left"], marker_dict, parameters=param_markers
        )
        
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, K_left, D_left
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    frame_copy["left"], [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(frame_copy["left"], K_left, D_left, rVec[i], tVec[i], 4, 4)
                print(f"id: {ids[0]} Dist: {round(distance, 2)} x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)}\n")
                cv2.putText(
                    frame_copy["left"],
                    f"id: {ids[0]} Dist: {round(distance, 2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    frame_copy["left"],
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        cv2.imshow(WINDOW_TITLE, frame_copy["left"])
        # cv2.imshow("P", gray_frame)

        if (wait):          
            print("i") 
            key = cv2.waitKey(1)
            if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                print("p")
                wait = False
        else:
            print("q")

        # Positional data frame
        pose = frames.get_pose_frame()
        if pose:
            pose_data = pose.get_pose_data()
            print("\nFrame number: %5.0f" % (pose.frame_number))
            print("Position xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.translation.x, pose_data.translation.y, pose_data.translation.z))
            print("Velocity xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z))
            print("Accelera xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z))
            print("Quatern xyzw: % 2.4f % 2.4f % 2.4f % 2.4f" % (pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w))
finally:
    pipe.stop()