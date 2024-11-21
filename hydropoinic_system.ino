#include <DHT.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPIO Pin Definitions
#define LDRPin 23         // GPIO where the LDR (light sensor) is connected
#define ventilateur 16    // GPIO where the fan is connected
#define Ec 18             // GPIO where the EC (electrical conductivity) sensor is connected
#define PH 32             // GPIO where the pH sensor is connected
#define lampe 19          // GPIO where the lamp is connected
#define pompeEau 2        // GPIO where the water pump is connected
#define pompeNutr 5       // GPIO where the nutrient pump is connected
#define TRIG_PIN1 23      // GPIO for ultrasonic sensor (water level) TRIG pin
#define ECHO_PIN1 22      // GPIO for ultrasonic sensor (water level) ECHO pin
#define TRIG_PIN2 21      // GPIO for ultrasonic sensor (nutrient level) TRIG pin
#define ECHO_PIN2 14      // GPIO for ultrasonic sensor (nutrient level) ECHO pin

// BLE Definitions
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char buffer[40];  // Buffer to store data for notification
bool rqsNotify;   // Request to notify the BLE client
unsigned long prvMillis; // Timestamp for notifications
#define INTERVAL_READ 1000  // Interval for reading sensors
int valNotify;
#define MAX_VALNOTIFY 600   // Maximum value for notification

// DHT Sensor for temperature and humidity
DHT dht(2, DHT11);  // DHT11 sensor on GPIO 2
float temp = 0;      // Variable to store temperature

// Ultrasonic sensor for water level
float waterLevel, distance1;
const float distanceCapteurWaterLevel = 10.00;  // Distance threshold for water level sensor
const float ReservoirWaterLevel = 20.00;        // Maximum water level in the reservoir

// Ultrasonic sensor for nutrient level
float NutrimentLevel, distance2;
const float distanceCapteurNutrimentrLevel = 10.00;  // Distance threshold for nutrient level sensor
const float ReservoirNutrimentLevel = 20.00;         // Maximum nutrient level in the reservoir

// Light sensor value
float light;

// Variables for pH, EC, and water temperature
float phValue;        // pH value (do not change)
float ecValue;        // EC value (do not change)
float TempdeauValue;  // Water temperature value (do not change)

// Function to calculate water level using ultrasonic sensor
float getWaterLevelValue() {
  float duration_us, distance_cm;
  digitalWrite(TRIG_PIN1, HIGH);  // Send pulse to TRIG pin
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);   // End pulse
  
  duration_us = pulseIn(ECHO_PIN1, HIGH);  // Measure pulse duration on ECHO pin
  distance_cm = 0.017 * duration_us;      // Calculate distance in cm
  return distance_cm;  // Return distance in cm
}

// Function to calculate nutrient level using ultrasonic sensor
float getNutrimentLevelValue() {
  float duration_us, distance_cm;
  digitalWrite(TRIG_PIN2, HIGH);  // Send pulse to TRIG pin
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);   // End pulse
  
  duration_us = pulseIn(ECHO_PIN2, HIGH);  // Measure pulse duration on ECHO pin
  distance_cm = 0.017 * duration_us;      // Calculate distance in cm
  return distance_cm;  // Return distance in cm
}

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID for BLE
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic UUID for RX
#define CHARACTERISTIC_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic UUID for TX
bool rqsNotify;  // Flag for notification request
unsigned long prvMillis;  // Timestamp for the last notification
#define INTERVAL_READ 1000  // Interval for reading sensor values

// Class for BLE server callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;  // Set the flag when the device is connected
      rqsNotify = false;       // Reset notification request
      prvMillis = millis();    // Set the timestamp for notifications
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false; // Set the flag when the device is disconnected
      rqsNotify = false;       // Reset notification request
      Serial.println("Device disconnected");
    }
};

// Function to process reading intervals and request notifications
void prcRead() {
  if(deviceConnected) {
    unsigned long curMillis = millis();
    if((curMillis - prvMillis) >= INTERVAL_READ) {
      rqsNotify = true;       // Set request for notification
      prvMillis = curMillis;  // Update the timestamp
    }
  }
}

// Setup function to initialize everything
void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(ventilateur, OUTPUT);  // Set fan pin as output
  pinMode(TRIG_PIN1, OUTPUT);    // Set ultrasonic sensor TRIG pin for water level as output
  pinMode(ECHO_PIN1, INPUT);     // Set ultrasonic sensor ECHO pin for water level as input
  pinMode(TRIG_PIN2, OUTPUT);    // Set ultrasonic sensor TRIG pin for nutrient level as output
  pinMode(ECHO_PIN2, INPUT);     // Set ultrasonic sensor ECHO pin for nutrient level as input
  pinMode(LDRPin, INPUT);        // Set LDR pin as input
  pinMode(lampe, OUTPUT);        // Set lamp pin as output
  pinMode(pompeEau, OUTPUT);     // Set water pump pin as output
  pinMode(pompeNutr, OUTPUT);    // Set nutrient pump pin as output
  dht.begin();                  // Initialize the DHT sensor
  digitalWrite(pompeNutr, LOW); // Turn off nutrient pump initially
  digitalWrite(pompeEau, LOW);  // Turn off water pump initially

  // Initialize BLE
  BLEDevice::init("ESP32");

  // Create BLE server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service and characteristic
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_WRITE  |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                      );
  pCharacteristic->addDescriptor(new BLE2902());  // Add BLE descriptor

  // Start the BLE service and advertising
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // Set to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting for client connection to notify...");
}

// Main loop function
void loop() {
  // Read EC sensor value
  ecValue = ((analogRead(Ec)) * 4.0) / 4095.0;
  Serial.print("ecValue: ");
  Serial.print(ecValue);
  Serial.println(" mS/cm");

  // Nutrient pump control based on EC value
  if (ecValue < 1.5) {
    digitalWrite(pompeNutr, HIGH); // Turn on nutrient pump
    delay(500);  
    digitalWrite(pompeNutr, LOW);  // Turn off nutrient pump
  }
  
  // Read pH sensor value
  phValue = ((analogRead(PH)) * 8.0) / 4095.0;
  Serial.print("phValue: ");
  Serial.println(phValue);

  // Water pump control based on pH value
  if (phValue < 6.0) {
    digitalWrite(pompeEau, HIGH); // Turn on water pump
    delay(500);  
    digitalWrite(pompeEau, LOW);  // Turn off water pump
  }

  // Read water level using ultrasonic sensor
  distance1 = getWaterLevelValue();
  waterLevel = ((ReservoirWaterLevel - (distance1 - distanceCapteurWaterLevel)) * 100) / ReservoirWaterLevel;
  Serial.print("WaterLevel: ");
  Serial.print(waterLevel);
  Serial.println("%");

  // Read nutrient level using ultrasonic sensor
  distance2 = getNutrimentLevelValue();
  NutrimentLevel = ((ReservoirWaterLevel - (distance2 - distanceCapteurNutrimentrLevel)) * 100) / ReservoirNutrimentLevel;
  Serial.print("NutrimentLevel: ");
  Serial.print(NutrimentLevel);
  Serial.println("%");

  // Read light level (LDR sensor)
  light = analogRead(LDRPin);
  Serial.print("Light: ");
  Serial.println(light);

  // Read temperature and humidity from DHT sensor
  temp = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" °C");

  // Send sensor data to BLE
  if (rqsNotify) {
    snprintf(buffer, sizeof(buffer), "Temp: %.2f, Water: %.2f%%, Nutrient: %.2f%%, EC: %.2f, pH: %.2f", temp, waterLevel, NutrimentLevel, ecValue, phValue);
    pCharacteristic->setValue(buffer);  // Set the value of the characteristic
    pCharacteristic->notify();          // Notify the connected client with the new data
    rqsNotify = false;                 // Reset the request flag
  }

  delay(1000);  // Delay before reading the sensors again
}
