import cv2

# OPEN CV 

# URL of the camera stream
stream_url = 'http://10.16.46.11:1183/stream.mjpg' # Replace this with the URL of the camera stream 10.16.46.11:????

# Open the video stream
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame")
        break

    # Draw some lines on the frame
    height, width, _ = frame.shape
    #cv2.line(frame, (0, 0), (width, height), (0, 255, 0), 2)  # Diagonal line from top-left to bottom-right
    #cv2.line(frame, (0, height), (width, 0), (0, 255, 0), 2)  # Diagonal line from bottom-left to top-right
    cv2.line(frame, (150, 0), (150, height), (0, 255, 0), 2)  # Vertical line in the middle
    cv2.line(frame, (175, 0), (175, height), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Camera Stream', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()