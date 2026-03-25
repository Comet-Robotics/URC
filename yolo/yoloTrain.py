from ultralytics import YOLO
import cv2
import numpy as np

def main():
    model = YOLO("yolov8n.pt")

    model.train(
        data="./data.yaml",
        epochs=50,
        imgsz=640,
        batch=16
    )

    model = YOLO("runs/detect/train/weights/best.pt")
    results = model("test_image3.jpg")

    img = results[0].plot()
    cv2.imshow("Result", img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    usr_response = input("Would you like to export this model :: ")
    if "y" in usr_response.lower():
        model.export(format="onnx")

if __name__ == '__main__':
    main()