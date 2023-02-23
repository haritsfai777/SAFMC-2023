# First import the library
from disEst import poseEstim, callback
import pyrealsense2 as rs

def threshold(x,y):
	if (abs(x) <= 5 and abs(y) <= 5):
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
    lastId = 0

    # lastId = 2
    bannedId = []

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and stream everything
    cfg = rs.config()

    # Start streaming with our callback
    pipe.start(cfg, callback)

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

      print("Switching to final markers")
      # Second loop from special dictionary command last special id
      # Loop the id with increment

      # untuk testing jangan lupa nanti diganti dengan yang dicomment di bawah
      currId = 1 # First id of special command 
      lastIdSp = 2 # Last id of special command

      # currId = 100 # First id of special command 
      # lastIdSp = 104 # Last id of special command
      while (True):
        x, y, idDistArr, id, command = poseEstim(currId, bannedId, IdDistArr, streams, isNewId)

        # Cek threshold
        if threshold(x,y):
          # Jika id yang sekarang bukan id terakhir
          if (currId != lastIdSp):
              print("Change next id")
              currId+=1
          
          # jika id yang sekarang adalah id terakhir, maka commandnya hanya UP
          else :
            print("Up")
            command = "UP"

    finally:
      print("Selesai")
      pipe.stop()

if __name__ == "__main__":
	arucoFollower()