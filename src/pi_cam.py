import cv2

# initialise the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("cannot open camera")
    exit()

while True:
    # capture frame by frame
    ret, frame = cap.read()

    if not ret:
        print("cannot read frame")
        break

    # display the frame called pi_cam_2_feed
    cv2.imshow("pi_cam_2_feed", frame)

    # press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# release the camera and close the window
cap.release()
cv2.destroyAllWindows()