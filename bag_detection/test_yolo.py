import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import message_filters
import json
import threading
from rclpy.executors import ExternalShutdownException

# Percorso dell'immagine
image_path = "/home/stefan/ros2_ws/src/bag_detection/bag_image/7.jpeg" 

# Percorso del modello YOLO
model_path = "/home/stefan/ros2_ws/src/bag_detection/detection_model/best.pt"
model = YOLO(model_path)

def main():

    rclpy.init()

    # Carica e processa l'immagine
    image = cv2.imread(image_path)
    if image is None:
        print(f"Errore: Immagine non trovata in {image_path}")
    else:
        results = model(image)
    
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]
                cls = int(box.cls[0])
                label = f"{model.names[cls]} {conf:.2f}"
    
                cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Elaborazione dei keypoints (se presenti)
            if hasattr(result, 'keypoints') and result.keypoints is not None:
                for keypoints in result.keypoints:
                    keypoint_coords = keypoints.xy[0].cpu().numpy()
                    print(f"keypoint_coords: {keypoint_coords}, shape: {keypoint_coords.shape}")
                    if keypoint_coords.size < 2:  # Controllo validità keypoints
                        continue
                    
                    # Disegna keypoints sull'immagine RGB
                    for kp in keypoint_coords:
                        x, y = int(kp[0]), int(kp[1])
                        cv2.circle(image, (x, y), 5, (0, 0, 255), -1)

        cv2.imshow("YOLOv8 Detection", image)
        if cv2.waitKey(0) & 0xFF == ord('q'):
            cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
