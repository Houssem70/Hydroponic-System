#include <DHT.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPIO where the ldr is connected to
#define LDRPin 23  
// GPIO where the DS18B20 is connected to
#define ventilateur 16
// GPIO where the EC is connected to
#define Ec  18
// GPIO where the PH is connected to
#define PH  32
// GPIO where the Lampe is connected to
#define lampe  19
// GPIO where the water pump is connected to
#define pompeEau  2
// GPIO where the Nutriment pump is connected to
#define pompeNutr  5
// GPIO where the Ultrason water is connected to
#define TRIG_PIN1 23 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN1 22 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin
// GPIO where the Ultrason nutriment is connected to
#define TRIG_PIN2 21 // ESP32 pin GIOP23 connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN2 14 // ESP32 pin GIOP22 connected to Ultrasonic Sensor's ECHO pin

//BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
char buffer[40];
//dht11
DHT dht(2, DHT11);
float temp = 0;

//ultrason1
float waterLevel,distance1;
const float  distanceCapteurWaterLevel = 10.00;
const float  ReservoirWaterLevel = 20.00;

//ultrason2
float NutrimentLevel,distance2;
const float  distanceCapteurNutrimentrLevel = 10.00;
const float  ReservoirNutrimentLevel = 20.00;

float light;


float phValue;// do not change
float ecValue;// do not change
float TempdeauValue;// do not change


// function to calculate water level

float getWaterLevelValue(){
  float duration_us, distance_cm;
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN1, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN1, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  return distance_cm;

}

// function to calculate nutriment level

float getNutrimentLevelValue(){
  float duration_us, distance_cm;
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(ECHO_PIN2, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  return distance_cm;

}


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
bool rqsNotify;
unsigned long prvMillis;
#define INTERVAL_READ 1000
int valNotify;
#define MAX_VALNOTIFY 600

int ain = 0;

//class for callback ble
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;

      rqsNotify = false;
      prvMillis = millis();
      Serial.println("Device connected");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      rqsNotify = false;
      Serial.println("Device disconnected");
    }
};

void prcRead(){
  if(deviceConnected){
    unsigned long curMillis = millis();
    if((curMillis-prvMillis) >= INTERVAL_READ){
           
      /*  int  g_Humidity = 565;//g_dht.readHumidity();
        Serial.print("Humidity: ");
        Serial.print(g_Humidity);
        Serial.println(" %");
        Serial.println(" ");
    
     // valNotify = map(g_Humidity, 0, 4096, 0, 255);
     // Serial.println(valNotify);
      */
      rqsNotify = true;
      prvMillis = curMillis;
    }
  }
  
}


//////////////////////////////////void stup ()////////////////////////////////////////////////////////
void setup() {
Serial.begin(9600);
pinMode(ventilateur,OUTPUT);
pinMode(TRIG_PIN1, OUTPUT);
pinMode(ECHO_PIN1, INPUT);
pinMode(TRIG_PIN2, OUTPUT);
pinMode(ECHO_PIN2, INPUT);
pinMode(LDRPin, INPUT);
pinMode(lampe, OUTPUT);
pinMode(pompeEau, OUTPUT);
pinMode(pompeNutr, OUTPUT);
dht.begin();
digitalWrite(pompeNutr,LOW);
digitalWrite(pompeEau,LOW);

BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}




///////////////////////////////void loop()////////////////////////////////////////////////////////////
void loop() {
  
 // read the input of ec from potenttiometer:
  ecValue = ((analogRead(Ec))*4.0)/4095.0;
  Serial.print("ecValue:");
  Serial.print(ecValue);
  Serial.println("mS/cm");
  
  //pompe de nutriment
  if (ecValue < 1.5 )
  {
   digitalWrite(pompeNutr,HIGH);
   delay(500);  
   digitalWrite(pompeNutr,LOW);
  }
  
  //read the input of ph from potenttiometer:
  phValue = ((analogRead(PH))*8.0)/4095.0;
  Serial.print("phValue:");
  Serial.println(phValue);

  
  //pompe d'eau 
  if (phValue < 6.0 )
  {
    digitalWrite(pompeEau,HIGH);
    delay(500);  
    digitalWrite(pompeEau,LOW); 
  }
  
// read water level from Ultrason
distance1 = getWaterLevelValue();
waterLevel = ((ReservoirWaterLevel-(distance1 - distanceCapteurWaterLevel))*100)/ReservoirWaterLevel;
Serial.print("WaterLevel: ");
Serial.print(waterLevel);
Serial.println("%");


// read nutriment level from Ultrason
distance2 = getNutrimentLevelValue();
NutrimentLevel = ((ReservoirWaterLevel-(distance2 - distanceCapteurNutrimentrLevel))*100)/ReservoirNutrimentLevel;
Serial.print("NutrimentrLevel: ");
Serial.print(NutrimentLevel);
Serial.println("%");


//read luminosity from ldr 
light = analogRead(LDRPin);
Serial.print(light);
Serial.println("LUX");

if( light< 200 )
{
  digitalWrite(lampe,HIGH);
}
else {
  digitalWrite(lampe,LOW);
}
 

  //Temperature d'air
  temp = dht.readTemperature();
  Serial.println();
  Serial.print("Température d'air: ");
  Serial.print(temp);
  Serial.print("°C");
  
 //ventiatur
 if(temp>28.00){
  digitalWrite(ventilateur,HIGH);
  }
 else {
digitalWrite(ventilateur,LOW);
  }
  

sprintf(buffer, "température %f, luminosité %f, EC %f ,PH %f, Niveau d'eau %f, Niveau de nutriments %f", temp, light, ecValue, phValue, waterLevel, NutrimentLevel);

// notify changed value
    if (deviceConnected) {
        if(rqsNotify){
          rqsNotify = false;
          pCharacteristic->setValue((char[])&buffer, 40);
          pCharacteristic->notify();
          //value++;
          
        }
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    prcRead();
    
delay(5000);
  
}
