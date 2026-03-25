import cv2
import numpy as np

class_names = [
    "person","bicycle","car","motorcycle","airplane","bus","train","truck","boat",
    "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
    "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite",
    "baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
    "wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich",
    "orange","broccoli","carrot","hot dog","pizza","donut","cake","chair","couch",
    "potted plant","bed","dining table","toilet","tv","laptop","mouse","remote",
    "keyboard","cell phone","microwave","oven","toaster","sink","refrigerator",
    "book","clock","vase","scissors","teddy bear","hair drier","toothbrush"
]

class_names = ['bottle', 'mallet', 'pick']

np.random.seed(42)
colors = np.random.randint(0, 255, size=(len(class_names), 3))
 
# Load ONNX model
net = cv2.dnn.readNet("best.onnx")

# Set backend (optional but recommended)
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

# Open video capture from camera (0 is default camera)
cap = cv2.VideoCapture(0)

# Set camera resolution and FPS (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

input_width = 640
input_height = 640
max_display_width = 1200  # clamp width
confidence_threshold = 0.5

print("Starting video feed. Press 'q' to quit...")

while True:
    ret, image = cap.read()
    # ret = True
    # image = cv2.imread("test_image.jpg")
    
    if not ret:
        print("Failed to read frame from camera")
        break
    
    height, width = image.shape[:2]

    # Preprocess (YOLOv8 uses 640x640 by default)
    blob = cv2.dnn.blobFromImage(
        image,
        scalefactor=1/255.0,
        size=(640, 640),
        swapRB=True,
        crop=False
    )

    net.setInput(blob)
    outputs = net.forward()

    predictions = np.squeeze(outputs)          # (84, 8400)
    predictions = predictions.T                # (8400, 84)

    boxes = []
    scores = []
    class_ids = []

    for pred in predictions:
        class_scores = pred[4:]
        class_id = np.argmax(class_scores)
        confidence = class_scores[class_id]

        if confidence > confidence_threshold:
            cx, cy, w, h = pred[0:4]

            x = int((cx - w/2) * width / input_width)
            y = int((cy - h/2) * height / input_height)
            w = int(w * width / input_width)
            h = int(h * height / input_height)

            boxes.append([x, y, w, h])
            scores.append(float(confidence))
            class_ids.append(class_id)

    indices = cv2.dnn.NMSBoxes(boxes, scores, 0.5, 0.4)

    for i in indices:
        i = i[0] if isinstance(i, (tuple, list, np.ndarray)) else i

        x, y, w, h = boxes[i]
        label = class_names[class_ids[i]]
        confidence = scores[i]

        text = f"{label} {confidence:.2f}"

        # Draw box
        color = [int(c) for c in colors[class_ids[i]]]
        cv2.rectangle(image, (x, y), (x+w, y+h), color, 2)

        # Draw label background
        (text_width, text_height), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
        )

        cv2.rectangle(
            image,
            (x, y - text_height - 4),
            (x + text_width, y),
            (0, 255, 0),
            -1
        )

        # Draw label text
        cv2.putText(
            image,
            text,
            (x, y - 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            1,
            cv2.LINE_AA
        )

    display_image = image.copy()
    h, w = display_image.shape[:2]

    if w > max_display_width:
        scale = max_display_width / w
        new_w = int(w * scale)
        new_h = int(h * scale)
        display_image = cv2.resize(display_image, (new_w, new_h))

    cv2.imshow("YOLOv8 Detection - Camera Feed", display_image)
    
    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()