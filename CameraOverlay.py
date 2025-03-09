import cv2

# Function to resize the image
def image_resize(image, width = None, height = None, inter = cv2.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    else:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))

    # resize the image
    resized = cv2.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized

# URL of the camera stream
stream_url = 'http://10.16.46.11:1182/stream.mjpg' # Replace this with the URL of the camera stream 10.16.46.11:?????
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

    # Resize the frame to fit the window using the image_resize function
    frame = image_resize(frame, width=window_width, height=800)

    # Draw some lines on the frame
    height, width, _ = frame.shape
    cv2.line(frame, (int(width*242/640), 0), (int(width*231/640), height), (0, 255, 0), 2)  # Vertical line in the middle
    cv2.line(frame, (int(width*196/640), 0), (int(width*187/640), height), (0, 255, 0), 2)

    # Display the resulting frame
    cv2.imshow('Camera Stream', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()