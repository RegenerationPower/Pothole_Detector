# Pothole Detector - A Smart City IoT System for Road Surface Monitoring

This project implements a system for detecting and analyzing road surface conditions using sensor data. The system aims to improve urban infrastructure by enabling the detection of potholes and irregularities, offering timely data collection for maintenance planning.

The system is built using an STM32 microcontroller (STM32F407G-DISC1) and a range of sensors like the MPU-6050 accelerometer and gyroscope, GPS module, temperature and humidity sensors. A neural network processes the collected data, automatically identifying road issues.

## Features
- **Real-time pothole detection** using machine learning.
- **Data collection** from multiple sensors (accelerometer, gyroscope, humidity, GPS).
- **Remote monitoring** and analysis via a Python-based user interface.
- **Comprehensive testing** for accurate data classification.
- **Visualization** of sensor data using Python libraries.

## Hardware and Software
### Hardware
- STM32F407G-DISC1
- MPU-6050 (accelerometer and gyroscope)
- GPS Module (GY-NEO6MV2)
- DHT11 (Temperature and Humidity sensor)
- LCD Display WH1602B-NYG-CT

### Software
- **Languages**: C (for STM32 firmware), Python (for data analysis and visualization)
- **Development Tools**: STM32CubeIDE, Python 3.8+
- **Libraries**: TensorFlow, matplotlib, and STM32 HAL libraries.

## Installation
1. **Clone the repository**:
   ```bash
   git clone https://github.com/RegenerationPower/Pothole_Detector.git
   cd Pothole_Detector
   ```

2. **Setup the hardware** by connecting the STM32 to sensors.
3. **Flash the STM32** with the provided firmware using STM32CubeIDE.
4. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

## How It Works
The STM32 microcontroller collects data from the sensors while mounted on a vehicle. This data is transmitted to a Python program, which feeds it into a neural network model. The neural network then identifies potholes and other irregularities on the road. Data visualization is provided through a GUI.

## Usage
The system operates in two modes: 
1. **Data Collection and Training Mode**: In this mode, the system collects sensor data, which is then used to train the neural network. This allows the model to learn how to detect potholes based on real-world data.
2. **Pothole Detection Mode**: Once the model is trained, this mode enables the system to identify potholes using the pre-trained model in real-time.

You can choose between these modes via the user interface.

### Steps:
1. **Select Mode**:
   - In the user interface, choose between the **Training Mode** or **Detection Mode**.
   
2. **For Data Collection and Training**:
   - Collect sensor data by driving a vehicle with the system installed.
   - Train the model using the collected data:

3. **For Pothole Detection**:
   - Run the pre-trained model to detect potholes:

4. **View Results**:
   - Monitor real-time sensor data and detection results through the interface.
