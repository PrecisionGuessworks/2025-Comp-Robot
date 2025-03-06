import cv2

# OPEN CV 

# URL of the camera stream
stream_url = 'http://localhost:1182/stream.mjpg' # Replace this with the URL of the camera stream 10.16.46.11:?????
#TEST: http://localhost:1182/stream.mjpg
#REAL: http://10.16.46.11:1183/stream.mjpg

# Open the video stream
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

cv2.namedWindow('Camera Stream', cv2.WINDOW_NORMAL)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame")
        break

    # Get the current window size
    window_width = cv2.getWindowImageRect('Camera Stream')[2]
    window_height = cv2.getWindowImageRect('Camera Stream')[3]

    # Resize the frame to fit the window
    frame = cv2.resize(frame, (window_width, window_height))

    # Draw some lines on the frame
    height, width, _ = frame.shape
    cv2.line(frame, (int(width*150/640), 0), (int(width*150/640), height), (0, 255, 0), 2)  # Vertical line in the middle
    cv2.line(frame, (int(width*175/640), 0), (int(width*175/640), height), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Camera Stream', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()