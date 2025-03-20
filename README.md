# Bag Detection - ROS 2

## 📌 Introduzione

Il progetto **Bag Detection** ha l'obiettivo di identificare e localizzare un sacco della spazzatura in un ambiente simulato utilizzando **YOLOv8** e un kinect sensor.

Il codice permette al robot di:

- Rilevare la posizione del sacco tramite YOLOv8.
- Individuare il nodo e di conseguenza la presa del sacco.
- Convertire le coordinate dell'oggetto in coordinate del mondo Gazebo.
- Pubblicare i risultati su un topic ROS 2.

---

## ⚙️ Requisiti

Per eseguire il progetto, assicurati di avere installato:

- **Ubuntu 22.04**
- **ROS 2 Humble** ([Guida all'installazione](https://docs.ros.org/en/humble/Installation.html))
- **Gazebo** ???
- **Python 3**

---

## 🚀 Installazione del Progetto

Clona il repository nella cartella `src` del tuo workspace ROS 2:

```
cd ~/ros2_ws/src
git clone https://github.com/raresstefan99/bag_detection
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Come avviare il progetto

### **1️⃣ Avviare la simulazione in Gazebo**

### **2️⃣ Avviare il nodo di rilevamento della bag**

```bash
ros2 run bag_detection camera_detection
```

### **3️⃣ Controllare i topic disponibili**

```bash
ros2 topic list
```

I topic creati sono:

```
/camera/color/image_raw
/camera/depth/image_raw
/keypoint_data
```

---

## 📂 Struttura del Codice

```bash
bag_detection/
├── bag_detection/                 # Script Python per il rilevamento della bag
│   ├── camera_detection.py
├── detection_model/                 # Modello YoloV8 per riconoscere la bag
│   ├── best.pt
├── models/                 # Modelli sdf della bag e della kinect
│   ├── bag
│   ├── kinect            
├── worlds/                # File sdf del mondo in Gazebo
│   ├── one_bagcopy
└── README.md               # Documentazione del progetto
```

---

## 🎯 Come funziona il rilevamento della bag

1. **Acquisizione dell'immagine**: la Kinect installata sul robot fornisce l'immagine RGB e la mappa di profondità.
2. **Elaborazione con YOLOv8**: lo script `camera_detection.py` utilizza YOLOv8 per rilevare la bag e identificare il keypoint del nodo.
3. **Trasformazione delle coordinate**:
   - Le coordinate dell'immagine vengono convertite in coordinate della camera.
   - Queste coordinate vengono poi trasformate in coordinate del sistema di riferimento del mondo Gazebo.
4. **Pubblicazione su ROS 2**: il nodo pubblica la posizione del nodo della bag nel topic `/keypoint_data`.

---

## 📧 Autore e Contatti

👤 **Autore**: [Rares Stefan Burnea]\
📧 **Email**: [burnearares@gmail.com]\
🌐 **GitHub**: [https://github.com/raresstefan99/bag_detection](https://github.com/raresstefan99/bag_detection)

Se hai problemi o suggerimenti, sentiti libero di contribuire! 🚀

