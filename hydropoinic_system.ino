#include <DHT.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPIO Pin Definitions
#define LIGHT_SENSOR_PIN 23         // GPIO where the Light Sensor (LDR) is connected
#define FAN_PIN 16                 // GPIO where the fan is connected
#define EC_SENSOR_PIN 18           // GPIO where the Electrical Conductivity (EC) sensor is connected
#define PH_SENSOR_PIN 32           // GPIO where the pH sensor is connected
#define LAMP_PIN 19                // GPIO where the lamp is connected
#define WATER_PUMP_PIN 2           // GPIO where the water pump is connected
#define NUTRIENT_PUMP_PIN 5        // GPIO where the nutrient pump is connected
#define WATER_LEVEL_TRIG_PIN 23    // GPIO for ultrasonic sensor TRIG pin (water level)
#define WATER_LEVEL_ECHO_PIN 22    // GPIO for ultrasonic sensor ECHO pin (water level)
#define NUTRIENT_LEVEL_TRIG_PIN 21 // GPIO for ultrasonic sensor TRIG pin (nutrient level)
#define NUTRIENT_LEVEL_ECHO_PIN 14 // GPIO for ultrasonic sensor ECHO pin (nutrient level)

// BLE Definitions
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool previousDeviceConnected = false;
char buffer[40];  // Buffer to store data for notification
bool notifyRequest;   // Request to notify the BLE client
unsigned long previousMillis; // Timestamp for notifications
#define READ_INTERVAL 1000  // Interval for reading sensors
int notificationValue;
#define MAX_NOTIFICATION_VALUE 600   // Maximum value for notification

// DHT Sensor for temperature and humidity
DHT dht(2, DHT11);  // DHT11 sensor on GPIO 2
float temperature = 0;      // Variable to store temperature

// Ultrasonic sensor for water level
float waterLevel, waterDistance;
const float waterLevelSensorThreshold = 10.00;  // Distance threshold for water level sensor
const float maxWaterLevel = 20.00;              // Maximum water level in the reservoir

// Ultrasonic sensor for nutrient level
float nutrientLevel, nutrientDistance;
const float nutrientLevelSensorThreshold = 10.00;  // Distance threshold for nutrient level sensor
const float maxNutrientLevel = 20.00;             // Maximum nutrient level in the reservoir

// Light sensor value
float lightSensorValue;

// Variables for pH, EC, and water temperature
float pHValue;        // pH value
float ECValue;        // EC value
float waterTemperatureValue;  // Water temperature value

// Function to calculate water level using ultrasonic sensor
float getWaterLevel() {
  float duration_us, distance_cm;
  digitalWrite(WATER_LEVEL_TRIG_PIN, HIGH);  // Send pulse to TRIG pin
  delayMicroseconds(10);
  digitalWrite(WATER_LEVEL_TRIG_PIN, LOW);   // End pulse
  
  duration_us = pulseIn(WATER_LEVEL_ECHO_PIN, HIGH);  // Measure pulse duration on ECHO pin
  distance_cm = 0.017 * duration_us;      // Calculate distance in cm
  return distance_cm;  // Return distance in cm
}

// Function to calculate nutrient level using ultrasonic sensor
float getNutrientLevel() {
  float duration_us, distance_cm;
  digitalWrite(NUTRIENT_LEVEL_TRIG_PIN, HIGH);  // Send pulse to TRIG pin
  delayMicroseconds(10);
  digitalWrite(NUTRIENT_LEVEL_TRIG_PIN, LOW);   // End pulse
  
  duration_us = pulseIn(NUTRIENT_LEVEL_ECHO_PIN, HIGH);  // Measure pulse duration on ECHO pin
  distance_cm = 0.017 * duration_us;      // Calculate distance in cm
  return distance_cm;  // Return distance in cm
}

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID for BLE
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic UUID for RX
#define CHARACTERISTIC_UUID    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic UUID for TX

// Class for BLE server callbacks
class BLEServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;  // Set the flag when the device is connected
      notifyRequest = false;       // Reset notification request
      previousMillis = millis();    // Set the timestamp for notifications
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false; // Set the flag when the device is disconnected
      notifyRequest = false;       // Reset notification request
      Serial.println("Device disconnected");
    }
};

// Function to process reading intervals and request notifications
void processReadings() {
  if(deviceConnected) {
    unsigned long currentMillis = millis();
    if((currentMillis - previousMillis) >= READ_INTERVAL) {
      notifyRequest = true;       // Set request for notification
      previousMillis = currentMillis;  // Update the timestamp
    }
  }
}

// Setup function to initialize everything
void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(FAN_PIN, OUTPUT);  // Set fan pin as output
  pinMode(WATER_LEVEL_TRIG_PIN, OUTPUT);    // Set ultrasonic sensor TRIG pin for water level as output
  pinMode(WATER_LEVEL_ECHO_PIN, INPUT);     // Set ultrasonic sensor ECHO pin for water level as input
  pinMode(NUTRIENT_LEVEL_TRIG_PIN, OUTPUT);    // Set ultrasonic sensor TRIG pin for nutrient level as output
  pinMode(NUTRIENT_LEVEL_ECHO_PIN, INPUT);     // Set ultrasonic sensor ECHO pin for nutrient level as input
  pinMode(LIGHT_SENSOR_PIN, INPUT);        // Set LDR pin as input
  pinMode(LAMP_PIN, OUTPUT);        // Set lamp pin as output
  pinMode(WATER_PUMP_PIN, OUTPUT);     // Set water pump pin as output
  pinMode(NUTRIENT_PUMP_PIN, OUTPUT);    // Set nutrient pump pin as output
  dht.begin();                  // Initialize the DHT sensor
  digitalWrite(NUTRIENT_PUMP_PIN, LOW); // Turn off nutrient pump initially
  digitalWrite(WATER_PUMP_PIN, LOW);  // Turn off water pump initially

  // Initialize BLE
  BLEDevice::init("ESP32");

  // Create BLE server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLEServerCallbacks());

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
  ECValue = ((analogRead(EC_SENSOR_PIN)) * 4.0) / 4095.0;
  Serial.print("ECValue: ");
  Serial.print(ECValue);
  Serial.println(" mS/cm");

  // Nutrient pump control based on EC value
  if (ECValue < 1.5) {
    digitalWrite(NUTRIENT_PUMP_PIN, HIGH); // Turn on nutrient pump
    delay(500);  
    digitalWrite(NUTRIENT_PUMP_PIN, LOW);  // Turn off nutrient pump
  }
  
  // Read pH sensor value
  pHValue = ((analogRead(PH_SENSOR_PIN)) * 8.0) / 4095.0;
  Serial.print("pHValue: ");
  Serial.println(pHValue);

  // Water pump control based on pH value
  if (pHValue < 6.0) {
    digitalWrite(WATER_PUMP_PIN, HIGH); // Turn on water pump
    delay(500);  
    digitalWrite(WATER_PUMP_PIN, LOW);  // Turn off water pump
  }

  // Read water level using ultrasonic sensor
  waterDistance = getWaterLevel();
  waterLevel = ((maxWaterLevel - (waterDistance - waterLevelSensorThreshold)) * 100) / maxWaterLevel;
  Serial.print("WaterLevel: ");
  Serial.print(waterLevel);
  Serial.println("%");

  // Read nutrient level using ultrasonic sensor
  nutrientDistance = getNutrientLevel();
  nutrientLevel = ((maxWaterLevel - (nutrientDistance - nutrientLevelSensorThreshold)) * 100) / maxNutrientLevel;
  Serial.print("NutrientLevel: ");
  Serial.print(nutrientLevel);
  Serial.println("%");

  // Read light level (LDR sensor)
  lightSensorValue = analogRead(LIGHT_SENSOR_PIN);
  Serial.print("LightSensorValue: ");
  Serial.println(lightSensorValue);

  // Control lamp based on light value
  if (lightSensorValue < 500) {
    digitalWrite(LAMP_PIN, HIGH);  // Turn on lamp if light level is low
  } else {
    digitalWrite(LAMP_PIN, LOW);   // Turn off lamp if light level is sufficient
  }

  // Read temperature and humidity
  temperature = dht.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C  Humidity: ");
  Serial.println(dht.readHumidity());

  // Process BLE notifications
  processReadings();

  // Send data to BLE client if notification requested
  if (notifyRequest) {
    snprintf(buffer, sizeof(buffer), "Water Level: %.2f%%, Nutrient Level: %.2f%%", waterLevel, nutrientLevel);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();   // Send notification to BLE client
    Serial.println("Sending notification: Water Level and Nutrient Level");
    notifyRequest = false;       // Reset notification request
  }
}
