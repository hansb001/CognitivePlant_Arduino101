
/***************************************************************************
   External Libraries
 **************************************************************************/
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <CurieBLE.h>
#include "DHT.h"

/***************************************************************************
   Internet Connectivity Setup - Variables & Functions
 **************************************************************************/
char ssid[] = "<your-network>"; // your network SSID (name)
char pass[] = "<your-passwd>";  // your network password
int status = WL_IDLE_STATUS;    // the Wifi radio's status

// Initialize the WiFi client library
WiFiClient wifiClient;

/***************************************************************************
   MQTT Initialization
 **************************************************************************/
#define MQTT_SERVER "realtime.ngi.ibm.com"

// Initialize pubsubclient to publish sensor data
PubSubClient client;

/***************************************************************************
   BLE initialization
 **************************************************************************/
BLEPeripheral blePeripheral;          // BLE Peripheral Device (the board you're programming)
BLEService moistureService("180D");   // BLE Moisture Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic moistureChar("2A37",  // standard 16-bit characteristic UUID
                              BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                                                        // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                                                        // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

const int ledForBLE = 6;      // LED on D6 to indicate BLE connection
long previousMillis = 0;      // last time moisture was read (in ms)

/***************************************************************************
   Grove board sensor initialization
 **************************************************************************/
#define MOISTURE_SENSOR A0    // input pin for moisture sensor
#define DHT_SENSOR A1         // input pin for temp & humidity sensor
#define DHTTYPE DHT11         // type of DHT sensor (in this case DHT11)
#define LIGHT_SENSOR A2       // input pin for light sensor

int moistureValue = 0;        // variable to store data from moisture sensor A0
int lightSensorValue = 0;     // variable to store data from lightsensor A2

float temperature;            // variable used to store temperature
float humidity;               // variable used to store humidity

StaticJsonBuffer<200> jsonBuffer;
char sensCharArray[200];
JsonObject& sensorJSON = jsonBuffer.createObject();

DHT dht(DHT_SENSOR, DHTTYPE, 20); // Initializing dht client

/***************************************************************************
   Board setup
 **************************************************************************/
void setup()
{
  // init serial link for debugging
  Serial.begin(9600);

  // Connect to WiFi network
  setupWiFi();
  
  // Setup client to MQTT server via port 1883 over WiFi. 
  client = PubSubClient(MQTT_SERVER, 1883, wifiClient);

  if (!client.connected()) {
    connectToMQTT();
  }

  // Set a local name for the BLE device
  // This name will appear in advertising packets and can be used by remote devices to identify this BLE device.
  // The name can be changed but maybe be truncated based on space left in advertisement packet.
  blePeripheral.setLocalName("CognitivePlantSketch");
  blePeripheral.setAdvertisedServiceUuid(moistureService.uuid());  // Add the service UUID
  blePeripheral.addAttribute(moistureService);                     // Add the BLE moisture service
  blePeripheral.addAttribute(moistureChar);                        // Add the moisture characteristic

  // Now activate the BLE device. It will start continuously transmitting BLE
  // advertising packets and will be visible to remote BLE central devices until it receives a new connection.
  blePeripheral.begin();
  Serial.println("[INFO] Bluetooth device active, waiting for connections...");

  // Initialize LED to indicate BLE connection
  pinMode(ledForBLE, OUTPUT);

  // Initialize humidity and temp sensor
  dht.begin();
}

void loop()
{
  // Listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  if (central) {
    Serial.print("[INFO] Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // Connected to central, so turn on LED
    digitalWrite(ledForBLE, HIGH);

    // Send out welcome notification to the plant's visitor
    sayHelloToVisitor();

    // Check moisture measurement every second as long as the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 1 second passed, read the moisture sensor
      if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;
        readMoistureSensor();
        Serial.print("moisture value: ");
        Serial.println(moistureValue);
        const unsigned char moistureCharArray[2] = { 0, (unsigned char) moistureValue };
        moistureChar.setValue(moistureCharArray, 2);  // and update the moisture measurement characteristic
      }
    }

    // when the central disconnects, turn off the LED:
    digitalWrite(ledForBLE, LOW);

    Serial.print("[INFO] Disconnected from central: ");
    Serial.println(central.address());
  }

  // Read sensor data from Grove board
  refreshMeasurements();

  /***************************************************************************
    // Publish sensor data & wait for next measurement
  ***************************************************************************/
  sensorJSON.printTo(sensCharArray, sizeof(sensCharArray));
  client.publish("iot-2/cmd/sensordata/fmt/string", sensCharArray);

  delay(10000); // Wait for 10 seconds for the next measurement

  client.loop();
}

void setupWiFi() {
  // Check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("[ERROR] WiFi shield not present");
    // don't continue:
    while (true);
  }

  // Print WiFi shield firmware version
  Serial.print("[INFO] WiFi firmware version: ");
  Serial.println(WiFi.firmwareVersion());

  // Print WiFi MAC Address
  Serial.print("[INFO] WiFi shield's MAC address: ");
  
  // Print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("[INFO] Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    // Wait 10 seconds for connection:
    delay(10000);
  }

  // Print IP address received from DHCP server
  Serial.print("[INFO] Connected using IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("");  
}

void readMoistureSensor() {
  /***************************************************************************
    // Input pin A0: Read moisture sensor
  ***************************************************************************/
  moistureValue = analogRead(MOISTURE_SENSOR);
  sensorJSON["moisture"] = moistureValue;

}

void sayHelloToVisitor() {
  // If lost connection to MQTT broker, attempt to reconnect...
  if (!client.connected()) {
    Serial.println("[WARN] Oops....lost connection to MQTT!! Trying to reconnect");
    connectToMQTT();
  } else {
    client.publish("iot-2/cmd/sayhello/fmt/string", "Hi there, thanks for visiting me!! It makes me feel happy again.");
  }
}

void connectToMQTT() {
  // clientID
  if (client.connect("d:xelcdl:ARDUINO:iot-arduino-001")) {
    Serial.println();
    Serial.print("[INFO] Connected to MQTT broker ");
    Serial.println(MQTT_SERVER);
  }
}

void refreshMeasurements() {
  // If lost connection to MQTT broker, attempt to reconnect...
  if (!client.connected()) {
    Serial.println("[WARN] Oops....lost connection to MQTT!! Trying to reconnect");
    connectToMQTT();
  } else {
    readMoistureSensor();

    /***************************************************************************
      // Input pin A1: Read temp & humidity sensor
    ***************************************************************************/
    // Reading temperature or humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
    humidity    = dht.readHumidity();
    temperature = dht.readTemperature();

    // check if returns are valid, if they are NaN (not a number) then something went wrong!
    if (isnan(temperature) || isnan(humidity))
    {
      Serial.println("[ERROR] Failed to read from DHT");
    }
    else
    {
      sensorJSON["humidity"] = humidity;
      sensorJSON["temperature"] = temperature;
    }

    /***************************************************************************
      // Input pin A2: Read light sensor
    ***************************************************************************/
    lightSensorValue = analogRead(LIGHT_SENSOR);
    sensorJSON["light"] = lightSensorValue;
  }
}
