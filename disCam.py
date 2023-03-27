"""
This example shows how to use T265 intrinsics and extrinsics in OpenCV to
asynchronously compute depth maps from T265 fisheye images on the host.
T265 is not a depth camera and the quality of passive-only depth options will
always be limited compared to (e.g.) the D4XX series cameras. However, T265 does
have two global shutter cameras in a stereo configuration, and in this example
we show how to set up OpenCV to undistort the images and compute stereo depth
from them.
Getting started with python3, OpenCV and T265 on Ubuntu 16.04:
First, set up the virtual enviroment:
$ apt-get install python3-venv  # install python3 built in venv support
$ python3 -m venv py3librs      # create a virtual environment in pylibrs
$ source py3librs/bin/activate  # activate the venv, do this from every terminal
$ pip install opencv-python     # install opencv 4.1 in the venv
$ pip install pyrealsense2      # install librealsense python bindings
Then, for every new terminal:
$ source py3librs/bin/activate  # Activate the virtual environment
$ python3 t265_stereo.py        # Run the example
"""

# First import the library
import pyrealsense2 as rs

# Import OpenCV and numpy
import cv2
import numpy as np
from cv2 import aruco

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

def rVecToYaw(rVec):
    rot_mtx, _ = cv2.Rodrigues(rVec)
    yaw = np.arctan2(rot_mtx[1, 0], rot_mtx[0, 0])
    yaw_degrees = yaw * 180 / np.pi

    return yaw_degrees

def poseEstim():
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    print("a")
    # Build config object and stream everything
    cfg = rs.config()
    print("a")

    # Start streaming with our callback
    pipe.start(cfg,callback)
    print("a")

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

        while True:
            print("p")
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
                print(frame_data["left"].copy())
                frame_mutex.release()
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
                        # cv2.polylines(
                        #     frame_copy["left"], [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                        # )
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
                #         point = cv2.drawFrameAxes(frame_copy["left"], K_left, D_left, rVec[i], tVec[i], 4, 4)
                #         print(f"id: {ids[0]} Dist: {round(distance, 2)} x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)}\n")
                #         cv2.putText(
                #             frame_copy["left"],
                #             f"id: {ids[0]} Dist: {round(distance, 2)}",
                #             top_right,
                #             cv2.FONT_HERSHEY_PLAIN,
                #             1.3,
                #             (0, 0, 255),
                #             2,
                #             cv2.LINE_AA,
                #         )
                #         cv2.putText(
                #             frame_copy["left"],
                #             f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                #             bottom_right,
                #             cv2.FONT_HERSHEY_PLAIN,
                #             1.0,
                #             (0, 0, 255),
                #             2,
                #             cv2.LINE_AA,
                #         )

                # cv2.imshow(WINDOW_TITLE, frame_copy["left"])
                # cv2.imshow("P", gray_frame)

                
            key = cv2.waitKey(1)
            # if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
            if key == ord('q'):
            # if key == ord('q'):
                break

    finally:
        pipe.stop()


if __name__ == "__main__":
    poseEstim()