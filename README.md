# Smart Hydroponic System - ESP32 Firmware

## Project Description

This project involves the development of a **Smart Hydroponic System** using an **ESP32** microcontroller. The system monitors and controls various parameters of a hydroponic setup, including temperature, humidity, pH levels, electrical conductivity (EC), water level, and light intensity. The system also uses actuators like pumps and fans to regulate environmental conditions.

The firmware, implemented on the ESP32, reads data from sensors, processes the data, and controls pumps and fans to optimize plant growth. Additionally, it communicates sensor data and system status updates over **Bluetooth Low Energy (BLE)**, allowing for remote monitoring via a mobile or PC application.

### Key Features:
- **Temperature and Humidity Monitoring**: Using a **DHT11** sensor and **LM35DZ** sensor.
- **Water and Nutrient Level Monitoring**: Using **Ultrasonic HC-SR04** sensors.
- **pH and EC Monitoring**: With **Analog pH Sensor/Meter Kit V2** and **Analog Electrical Conductivity Sensor/Meter V2**.
- **Environmental Control**: Controls the **fan**, **water pump**, and **nutrient pump** to maintain optimal conditions.
- **Light Intensity Monitoring**: Using an **LDR** sensor.
- **BLE Communication**: Real-time data access and remote control from a BLE-enabled device.

## Hardware Components

- **ESP32 Dev Board**: The main microcontroller for controlling the system.
- **Pompe JT500 DC12V**: Water pump used for circulating water through the hydroponic system.
- **Module Relais 1 channel 5V**: Used to control the pumps and fan.
- **DS18B20**: Digital temperature sensor.
- **LM35DZ**: Analog temperature sensor for measuring water temperature.
- **Ultrasonic HC-SR04**: For measuring the water level and nutrient solution level.
- **DHT11**: For measuring air temperature and humidity.
- **LDR (Light Dependent Resistor)**: For measuring light intensity (luminosity).
- **Analog pH Sensor/Meter Kit V2 (SEN0161-V2)**: For monitoring the pH level of the solution.
- **Analog Electrical Conductivity (EC) Sensor/Meter V2**: For measuring the electrical conductivity of the nutrient solution.

## Software Requirements

- **Arduino IDE**: For uploading the code to the ESP32.
- **ESP32 Board Package**: Install via the Arduino Board Manager.
- **BLE Library**: For Bluetooth communication.
- **DHT Sensor Library**: For DHT11 sensor.
- **OneWire Library**: For DS18B20 sensor.

## Installation

### Step 1: Setting up the Arduino IDE
1. Download and install [Arduino IDE](https://www.arduino.cc/en/software).
2. Open the Arduino IDE and go to **File > Preferences**.
3. In the **Additional Boards Manager URLs** field, add:
   https://dl.espressif.com/dl/package_esp32_index.json
4. Go to **Tools > Board > Board Manager**, search for "ESP32", and install the ESP32 package.
5. Select **ESP32 Dev Module** under **Tools > Board**.

### Step 2: Install Libraries
1. Open the **Library Manager** via **Sketch > Include Library > Manage Libraries**.
2. Install the following libraries:
- **DHT sensor library** (for DHT11 sensor).
- **BLE** (for Bluetooth communication).
- **OneWire** (for DS18B20 sensor).

### Step 3: Upload the Code
1. Download the project code from the repository or copy the provided code into a new sketch.
2. Connect the ESP32 board to your computer via USB.
3. Select the correct port under **Tools > Port**.
4. Click the **Upload** button to upload the code to the ESP32.

## Hardware Wiring

1. **DS18B20 Temperature Sensor**: Connect the **Data** pin to GPIO 4 of the ESP32. Use a 4.7kΩ pull-up resistor between **Data** and **VCC**.
2. **LM35DZ Temperature Sensor**: Connect the **Output** pin to GPIO 32 for analog reading.
3. **Ultrasonic HC-SR04 Sensors**:
- **Water Level Sensor**: Connect **TRIG** and **ECHO** to GPIO 23 and GPIO 22.
- **Nutrient Level Sensor**: Connect **TRIG** and **ECHO** to GPIO 21 and GPIO 19.
4. **DHT11**: Connect the **Data** pin to GPIO 2.
5. **LDR**: Connect one end of the LDR to 3.3V and the other to **GPIO 34**, with a 10kΩ pull-down resistor between GPIO 34 and GND.
6. **Analog pH Sensor**: Connect the **Analog Output** to GPIO 33.
7. **Analog EC Sensor**: Connect the **Analog Output** to GPIO 34.
8. **Pompe JT500 DC12V (Water Pump)**: Connect the **Relay** control pin to GPIO 15 for controlling the water pump.
9. **Nutrient Pump**: Connect the **Relay** control pin to GPIO 13 for controlling the nutrient pump.
10. **Fan**: Connect the **Relay** control pin to GPIO 12 for controlling the fan.

Refer to the wiring diagram in the repository (if provided) for a detailed setup.

## Usage

### BLE Communication
Once powered on, the ESP32 will advertise a **BLE service** with the following characteristics:
- **Service UUID**: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- **Characteristic UUID (TX)**: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`

A BLE-enabled device (smartphone or PC) can connect to the ESP32 to:
- **Receive live sensor data** including temperature, humidity, water level, nutrient level, pH, EC, and light intensity.
- **Control actuators**: Remotely turn on/off the water pump, nutrient pump, and fan.

### Real-Time Monitoring
The system will continuously monitor the sensor values. If the **water level** or **nutrient level** falls below a set threshold, the system will activate the corresponding pump. Similarly, the **pH** and **EC** sensors will trigger the pumps as needed.

### Notifications
The ESP32 will send periodic notifications with the following sensor data to the connected BLE device:
- Temperature (°C)
- Humidity (%)
- Water Level (%)
- Nutrient Level (%)
- EC (mS/cm)
- pH Value
- Light Intensity (Lux)

### Control
The BLE app allows for manual control of the **water pump**, **nutrient pump**, and **fan**. 

## Troubleshooting

- **No BLE Connection**: Ensure your device is within BLE range and has Bluetooth enabled.
- **Incorrect Sensor Readings**: Check the wiring of the sensors and ensure the analog sensors are connected to the correct GPIO pins.
- **Device Not Responding**: Reset the ESP32 and verify sensor connections.

