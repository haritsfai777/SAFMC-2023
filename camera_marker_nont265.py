import cv2 as cv
from cv2 import aruco
import numpy as np

marker_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)

param_markers = aruco.DetectorParameters_create()

cap = cv.VideoCapture(2)
camera_matrix = np.array([[578.92810113,   0,         348.69375095]
, [  0,         580.59503862, 187.37399863]
, [  0,           0,           1.        ]])
dist_coeffs = np.array([[-0.41518529,  0.27491043, -0.00658387, -0.00610845, -0.17400443]])

while True:
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    marker_corners, marker_IDs, reject = aruco.detectMarkers(
        frame, marker_dict, parameters=param_markers
    )
    if marker_corners:
        _, tVec, _ = aruco.estimatePoseSingleMarkers(
            marker_corners, 16.5, camera_matrix, dist_coeffs
        )

        total_markers = range(0, marker_IDs.size)

        for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
            cv.polylines(
                frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
            )
            corners = corners.reshape(4, 2)
            corners = corners.astype(int)
            top_right = corners[0].ravel()
            top_left = corners[1].ravel()
            bottom_right = corners[2].ravel()
            bottom_left = corners[3].ravel()
            x = round(tVec[i][0][0],1)
            y = round(tVec[i][0][1],1)

            print(f'x: {x}, y: {y}')
            print(f"corner {corners[1]}")
            cv.putText(
                frame,
                f"id: {ids[0]}",
                top_right,
                cv.FONT_HERSHEY_PLAIN,
                1.3,
                (200, 100, 0),
                2,
                cv.LINE_AA,
            )
            # print(ids, "  ", corners)
    cv.imshow("frame", frame)
    key = cv.waitKey(1)
    if key == ord("q"):
        break
cap.release()
cv.destroyAllWindows()
