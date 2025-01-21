import cv2
import cvlib as cv
from cvlib.object_detection import draw_bbox # this will draw a box around the object


video = cv2.VideoCapture(0) # 0 is the default camera

# video.set(cv2.CAP_PROP_FRAME_WIDTH, 320)  # set width to 320 pixels
# video.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # set height to 240 pixels

frame_skip = 2  # Process every 2nd frame
frame_count = 0

labels = []

while video.isOpened():
    ret, frame = video.read()
    bbox, label, conf = cv.detect_common_objects(frame) # this will detect the object in the frame
    output_image = draw_bbox(frame, bbox, label, conf) # this will draw a box around the object

    frame_count += 1

    if frame_count % frame_skip != 0:
        continue  # Skip frames

    cv2.imshow("Object Detection from pi cam", output_image)

    # to add something that the camera detects to the list but only once
    for item in label:
        if item in labels:
            pass
        else:
            labels.append(item)
            print(labels)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video.release()
cv2.destroyAllWindows()

