# **Automated Robotic Arm for Hole Punching**  

This repository contains the complete codebase for an **automated robotic arm** designed for hole-punching tasks in industrial settings. The system integrates **Arduino-controlled pneumatic mechanisms**, **computer vision with YOLO**, and **robotic inverse kinematics** to automate precise hole punching in paper and plastic materials.  

## **Project Overview**  

The robotic system consists of:  
- **Arduino-controlled pneumatic punching** mechanism that receives angle commands from the Raspberry Pi.  
- **Computer vision** for detecting objects and determining punch locations.  
- **Inverse kinematics** for robotic arm movement.  
- **Camera calibration** for accurate object localization.  

## **Folder Structure**  

📂 **arduino/**  
   - `arduino_logic.ino` → Controls pneumatic machine operations, receives angles from Raspberry Pi, and executes movements.  

📂 **calibration/**  
   - `intrinsics.json` → Camera intrinsic parameters after calibration:  
     ```json
     {
        "fx": 487.1749,
        "fy": 487.7741,
        "cx": 323.347,
        "cy": 238.453537,
        "distortion": [0.075920796579382, -0.041364902728913, 0, 0, 0]
     }
     ```  

📂 **computer_vision/**  
   - `object_detection.py` → Runs on Raspberry Pi for real-time object detection.  
   - `yolo_benchmark.ipynb` → Evaluates YOLO model performance.  
   - `yolo_train_convert.ipynb` → Converts and trains YOLO for object detection.
   - `packerOut` → is used for creating .rpk computer vision model.

📂 **robot_block/**  
   - `inverse_kinematics/` → Contains:  
     - `inverse_kinematic.py` → Executes inverse kinematics on Raspberry Pi for robotic arm movement.  
   - `robot.urdf` → Defines the robotic arm structure.  

## **Installation & Usage**  

### **1️⃣ Setup Arduino**  
Upload `arduino_logic.ino` to your **Arduino** board. This handles pneumatic punching and receives movement commands.  

#### **Prerequisites**  
This project is designed to run on a **Raspberry Pi 4 or Raspberry Pi 5** with a compatible **AI camera**. If using other Raspberry Pi models with a camera connector (e.g., Raspberry Pi Zero 2 W, Raspberry Pi 3 Model B+), minor modifications may be needed.  


### **2️⃣ Setup AI camera on Raspberry Pi**  
Install IMX500 Firmware (for AI Camera Users)
To enable AI processing on the camera, install the necessary firmware:
```sudo apt install imx500-all imx500-tools```

This command:

Installs required firmware files for the IMX500 sensor.
- **Adds neural network model firmware files to /usr/share/imx500-models/.**
- **Installs IMX500 post-processing software.**
- **Installs Sony's network model packaging tools.**

Install Required Dependencies.To run the picamera2 application (used for model deployment), install:


```sudo apt install python3-opencv python3-munkres```

Once all dependencies are installed, restart your Raspberry Pi:

```sudo reboot```

### **3️⃣Package and Deploy AI Model to the Camera**  


After obtaining packerOut.zip from the IMX500 conversion process this zip file can be found in computer_vision folder , you need to package it into an RPK file for deployment to the AI camera.

Run the following command to convert the zip file into a deployable model file:
```imx500-package -i <path to packerOut.zip> -o <output folder>```
This will generate a network.rpk file inside the specified output folder.


### **4️⃣ Run Object Detection on Raspberry Pi**  

Create virtual eniroment in folder that contains ```inverse_kinematic.py```, ```object_detection.py``` ```robot_file.urdf```, ```intrinsics.json``` and ```network.rpk```.

Install all libraries and run this comand:
```python object_detection.py``` 

## About object_detection.py
The original object detection script was taken from [imx500_object_detection_demo.py](https://github.com/raspberrypi/picamera2/blob/main/examples/imx500/imx500_object_detection_demo.py) in the [IMX500 Raspberry Pi repository](https://github.com/raspberrypi/imx500-models.git) and has been modified for this project.

### Credits:
- **Original code**: [IMX500 Repository](https://github.com/raspberrypi/picamera2/blob/main/examples/imx500/imx500_object_detection_demo.py)  
- **Original license**: MIT License  
- **Modified by**: Matvejs Aleksejevs  
- **Changes made**:  
  - [List of modifications, "Center detection", "Pixel coordinate transformation into robot base coordinates"]

### License:
This project remains under the MIT License. See the `LICENSE` file for details.
