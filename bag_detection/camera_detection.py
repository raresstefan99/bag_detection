#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import message_filters
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
#import threading

class BagDetectionNode(Node):
    """
    Nodo ROS per il rilevamento di oggetti con YOLOv8.
    """

    def __init__(self):
        super().__init__('bag_detection_node')
        
        # Inizializza il modello YOLOv8
        model_path = '/home/stefan/ros2_detection_ws/src/bag_detection/detection_model/best.pt'  # Percorso del modello YOLOv8
        self.model = YOLO(model_path)

        # Per convertire messaggi ROS in immagini OpenCV
        self.bridge = CvBridge()

        # Parametri della videocamera Kinect
        self.width = 640  # Larghezza dell'immagine RGB
        self.height = 480  # Altezza dell'immagine RGB
        self.horizontal_fov = 1.0472  # Campo visivo orizzontale in radianti (60 gradi)
        self.focal_length = self.width / (2 * np.tan(self.horizontal_fov / 2))  # Calcolo della lunghezza focale in pixel

        # Posizione e orientamento della videocamera nel mondo Gazebo
        self.camera_position = np.array([-0.118124, 0.261756, 1.384410])  # Posizione della videocamera
        self.camera_orientation = np.array([-0.057092, 1.039411, 1.489529])  # Orientamento in radianti (roll, pitch, yaw)

        # Calcolo della matrice di rotazione della videocamera
        self.camera_rotation_matrix = R.from_euler('xyz', self.camera_orientation, degrees=False).as_matrix()

        # Publisher per i keypoints rilevati
        self.keypoint_pub = self.create_publisher(String, 'keypoint_data', 10)
        
        # Sottoscrizione ai topic RGB e Depth
        self.rgb_sub = Subscriber(self, Image, '/camera/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/camera/depth/image_raw')
        
        # Sincronizzatore per unire i messaggi RGB e Depth
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.image_callback)


        self.get_logger().info("Nodo Bag Detection avviato!")



    def spin_in_background():
        """
        Questa funzione permette di eseguire la callback fino a quando qualcosa lo spegne.
        """
        executor = rclpy.get_global_executor()
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass


    def transform_to_world(self, x_cam, y_cam, z_cam):
        """
        Trasforma le coordinate della videocamera in coordinate del mondo.
        """
        # Coordinate relative alla videocamera
        camera_coords = np.array([x_cam, y_cam, z_cam])
        # Trasformazione nello spazio del mondo
        world_coords = self.camera_position + self.camera_rotation_matrix.dot(camera_coords)
        return world_coords

    def detect_objects(self, rgb_image, depth_image):
        """
        Esegue il rilevamento degli oggetti sull'immagine RGB e calcola le coordinate 3D nel mondo.
        """

        # Esegui il rilevamento con YOLOv8
        results = self.model(rgb_image)

        for result in results:
            # Elaborazione dei bounding box
            for box in result.boxes:
                # Coordinate del bounding box
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = box.conf[0]  # Confidenza del rilevamento
                cls = int(box.cls[0])  # Classe dell'oggetto rilevato
                label = f"{self.model.names[cls]} {conf:.2f}"  # Etichetta dell'oggetto
            
            
                # Estrai la regione di interesse (ROI) dalla mappa di profondità
                depth_roi = depth_image[y1:y2, x1:x2]
                mean_depth = np.nanmean(depth_roi)  # Calcola la profondità media nella ROI
                if np.isnan(mean_depth) or mean_depth <= 0 or mean_depth > 10.0:  # Controllo validità profondità
                    print("Profondità non valida o non disponibile.")
                    continue

                # Disegna il bounding box sull'immagine RGB
                cv2.rectangle(rgb_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(rgb_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                print(f"Oggetto rilevato: {label}, Profondità media: {mean_depth:.2f} m")

            if hasattr(result, 'keypoints') and result.keypoints is not None:
                self.process_keypoints(result.keypoints, depth_image, rgb_image)
        
        return rgb_image

    def process_keypoints(self, keypoints, depth_image, rgb_image):
        """
        Elabora i keypoints rilevati e calcola le coordinate 3D nel mondo.
        """
        keypoint_coords = keypoints.xy[0].cpu().numpy()
        print(f"keypoint_coords: {keypoint_coords}, shape: {keypoint_coords.shape}")
        if keypoint_coords.size < 2:  # Controllo validità keypoints
            return
            
        for kp in keypoint_coords:
            # Estrai le coordinate del primo keypoint
            keypoint_x, keypoint_y = keypoint_coords[0, 0], keypoint_coords[0, 1]
            if not (0 < keypoint_x < self.width and 0 < keypoint_y < self.height):
                print(f"Keypoint fuori dai limiti: X={keypoint_x}, Y={keypoint_y}")
                continue        
            

             # Mi asicuro che le coordinate siano intere
            keypoint_x, keypoint_y = int(keypoint_x), int(keypoint_y)

            keypoint_y = 480 - keypoint_y	# Ribalto l'asse y perche CV ha l'origine in alto a sinistra con asse y verso il basso invece YOLO in basso a sinistra con asse y verso l'alto
            
            # Profondità al keypoint
            keypoint_depth = depth_image[keypoint_y, keypoint_x]
            if np.isnan(keypoint_depth) or keypoint_depth <= 0:
                print("Profondità del keypoint non valida.")
                continue

            # Calcola le coordinate relative alla videocamera
            relative_x = (keypoint_x - self.width / 2) / (self.width / 2)
            relative_y = (keypoint_y - self.height / 2) / (self.height / 2)
            keypoint_x_cam = relative_x * keypoint_depth * np.tan(self.horizontal_fov / 2)
            keypoint_y_cam = relative_y * keypoint_depth * np.tan(self.horizontal_fov / 2)
            keypoint_z_cam = keypoint_depth

            world_coords = self.transform_to_world(keypoint_x_cam, keypoint_y_cam, keypoint_z_cam)

            # Pubblica i dati del keypoint
            keypoint_data = json.dumps({
                "keypoints": [{
                    "x": world_coords[0],
                    "y": world_coords[1],
                    "z": world_coords[2]
                }]
            })
            self.keypoint_pub.publish(keypoint_data)
                
            # Disegna keypoints sull'immagine RGB
            for kp in keypoint_coords:
                x, y = int(kp[0]), int(kp[1])
                cv2.circle(rgb_image, (x, y), 5, (0, 0, 255), -1)

            self.get_logger().info(f"Keypoint: X={world_coords[0]:.2f}, Y={world_coords[1]:.2f}, Z={world_coords[2]:.2f}")


    def image_callback(self, rgb_msg, depth_msg):
        """
        Callback per acquisire immagini dalla videocamera Gazebo e applicare YOLOv8.
        """
        try:
            # Converte i messaggi ROS in immagini OpenCV
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
            self.get_logger().info("Ricevuto frame RGB e Depth!")
        except CvBridgeError as e:
            self.get_logger().error(f"Errore nella conversione dell'immagine: {e}")
            return

        # Applica YOLOv8
        processed_image = self.detect_objects(rgb_image, depth_image)

        # Mostra l'immagine elaborata
        cv2.imshow("YOLOv8 Detection", processed_image)
        cv2.waitKey(1)


def main():
    
    rclpy.init()
    # Inizializza il nodo ROS
    node = BagDetectionNode()

#    executor = rclpy.executors.MultiThreadedExecutor()
#    executor.add_node(node)
    
    # Avvia il nodo in un thread separato
#    thread = threading.Thread(target=executor.spin, daemon=True)
#    thread.start()

    # Publisher per i dati dei keypoints
    keypoint_pub = node.create_publisher(String, 'keypoint_data', 10)

    # Sincronizzazione dei messaggi RGB e Depth
#    ts = message_filters.ApproximateTimeSynchronizer([node.rgb_sub, node.depth_sub], 10, 0.1)
#    ts.registerCallback(lambda rgb_msg, depth_msg: node.image_callback(rgb_msg, depth_msg))

    node.get_logger().info("Nodo YOLOv8 avviato. In attesa di immagini...")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ferma il nodo e chiude la finestra di visualizzazione
        cv2.destroyAllWindows()
        node.destroy_node()
        node.get_logger().info("Interruzione del nodo YOLOv8.")
        rclpy.shutdown()
#        thread.join()
        node.get_logger().info("Nodo YOLOv8 terminato.")


if __name__ == "__main__":
    main()
