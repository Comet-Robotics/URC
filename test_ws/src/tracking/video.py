import cv2

STREAM_URL = "http://11.33.7.137:8080/video"

print(f"Connecting to {STREAM_URL}...")
cap = cv2.VideoCapture(STREAM_URL)

if not cap.isOpened():
    print("Failed to open stream.")
    exit(1)

print("Connected. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("No frame received.")
        break

    cv2.imshow("Webcam Stream (VLC MJPEG)", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
