import cv2

vid = cv2.VideoCapture("rtsp://172.26.160.1:8554/webcam.h264")

while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
    print(frame.shape)
  
    # Display the resulting frame
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()