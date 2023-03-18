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
    
    #   specCommandDict
  '100': 'LEFT', 
  '101': 'FORWARD',
  '102': 'RIGHT',
  '103': 'BACKWARD',
}

def poseEstim(currId, bannedId, IdDistArr, video_capture, isNewId):

    # Getting camera parameters
    width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = video_capture.get(cv2.CAP_PROP_FPS)
    camera_matrix = np.array([[578.92810113,   0,         348.69375095]
    , [  0,         580.59503862, 187.37399863]
    , [  0,           0,           1.        ]])
    dist_coeffs = np.array([[-0.41518529,  0.27491043, -0.00658387, -0.00610845, -0.17400443]])

    MARKER_SIZE = 16.5  # centimeters
    marker_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_50) # Aruco type
    param_markers = aruco.DetectorParameters_create()
    
    while True:
        ret, frame_copy = video_capture.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame_copy, cv2.COLOR_BGR2GRAY)

        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray, marker_dict, parameters=param_markers
        )
        
        if marker_corners:
            # Getting tVec from estimatePoseSingleMarkers 
            _, tvecs, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, camera_matrix, dist_coeffs
            )
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    frame_copy, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )

                distance = np.sqrt(
                    tvecs[i][0][2] ** 2 + tvecs[i][0][0] ** 2 + tvecs[i][0][1] ** 2
                )
                
                # if want to delete the key, just type del idDistArr[ids[i]]        

                # Draw the pose of the marker
                x = round(tvecs[i][0][0],1)
                y = round(tvecs[i][0][1],1)

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
    currId = 0
    bannedId=[]

    video_capture = cv2.VideoCapture(1)
    IdDistArr = {}

    poseEstim(currId, bannedId, IdDistArr, video_capture, False)

    # poseEstim(currId, bannedId)    
