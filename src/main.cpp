// #include <Arduino.h>
// #include "DHT20.h"
// // #include "DHT.h"
// // #include <Wire.h>


// #define SDA_PIN 11
// #define SCL_PIN 12

// // #define DHT22_PIN 4 // GPIO 4 for DHT22 data pin
// // #define DHTTYPE DHT22

// DHT20 dht20;
// // DHT dht(DHT22_PIN, DHTTYPE);



// void setup() {
//   Serial.begin(115200);
//   while(!Serial){};
//   delay(7000);
//   // while (!Serial)
//   //   ;
//   Wire.begin(SDA_PIN, SCL_PIN);

//   // Wire.begin();
//   if (!dht20.begin())
//   {
//     Serial.println("Không tìm thấy cảm biến DHT20!");
//     while (1);
//   }
//   Serial.println("DHT20 đã sẵn sàng.");
//   // Serial.println(dht20.read());
//   // delay(10000);
//   // dht.begin();
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   Serial.println("---");

//   dht20.read();
//   float temperature = dht20.getTemperature();
//   float humidity = dht20.getHumidity();
//   Serial.print("Temperature: ");
//   Serial.print(temperature);
//   Serial.print("°C, Humidity: ");
//   Serial.print(humidity);
//   Serial.println("%");

//   delay(3000);

// }

/*------------------------------------------------COREIOT------------------------------------------------*/
// library
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "DHT.h"
#include "DHT20.h"
#include <Shared_Attribute_Update.h>
#include <Attribute_Request.h>

#define SDA_PIN 11
#define SCL_PIN 12
#define RELAY1_PIN 2
#define RELAY2_PIN 4
#define RELAY3_PIN 5
#define RELAY4_PIN 18


DHT20 dht20;

const char SSID[] = "ACLAB";
const char PASSWORD[] = "ACLAB2023";

const char THINGSBOARD_SERVER[] = "app.coreiot.io";
const int THINGSBOARD_PORT = 1883;
const char ACCESS_TOKEN[] = "oxvfp3csvkoy7pzvtpwp";

const int maxMessageSize = 4096;

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard thingsBoard(mqttClient, maxMessageSize);

float temperature = 0.0;
float humidity = 0.0;

void setupRelays() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
}

void setRelayState(int relayNumber, bool state) {
  int pin;
  switch (relayNumber) {
    case 1: pin = RELAY1_PIN; break;
    case 2: pin = RELAY2_PIN; break;
    case 3: pin = RELAY3_PIN; break;
    case 4: pin = RELAY4_PIN; break;
    default: return;
  }
  digitalWrite(pin, state ? HIGH : LOW);
}

// --------------------- Shared Attributes Callback ---------------------
void processSharedAttributes(const JsonObjectConst &data) {
  Serial.println("Received Shared Attributes:");

  if (data.containsKey("relay1")) {
    bool state = data["relay1"];
    setRelayState(1, state);
    Serial.printf("Relay1 set to %s\n", state ? "ON" : "OFF");
  }
  if (data.containsKey("relay2")) {
    bool state = data["relay2"];
    setRelayState(2, state);
    Serial.printf("Relay2 set to %s\n", state ? "ON" : "OFF");
  }
  if (data.containsKey("relay3")) {
    bool state = data["relay3"];
    setRelayState(3, state);
    Serial.printf("Relay3 set to %s\n", state ? "ON" : "OFF");
  }
  if (data.containsKey("relay4")) {
    bool state = data["relay4"];
    setRelayState(4, state);
    Serial.printf("Relay4 set to %s\n", state ? "ON" : "OFF");
  }
}

void manageWifi(void *pvParameters){
  (void)pvParameters;

  while (true){
    if (WiFi.status() != WL_CONNECTED){
      Serial.println("Connecting to WiFi...");
      WiFi.begin(SSID, PASSWORD);

      int attempt = 0;
      while (WiFi.status() != WL_CONNECTED && attempt < 20){// gioi han 20 lan thu ket noi
        delay(500);
        Serial.print(".");
        attempt++;
      }

      if (WiFi.status() == WL_CONNECTED){
        Serial.println("\nWiFi connected");
      }
      else{
        Serial.println("\nFailed to connect to WiFi");
      }
    }
    else {
      Serial.println("Wifi already connected");
    }

    // kiem tra lai ket noi moi 10 giay
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void connectToCoreIoT(void *pvParameters)
{
  (void)pvParameters;

  while (true)
  {
    if (!thingsBoard.connected())
    {
      if (!thingsBoard.connect(THINGSBOARD_SERVER, ACCESS_TOKEN, THINGSBOARD_PORT))
      {
        Serial.println("Failed to connect to Core IoT");
      }
      else
      {
        Serial.println("Connected to Core IoT");
        thingsBoard.Shared_Attributes_Update(processSharedAttributes);
      }
    }
    else
    {
      Serial.println("Core IoT already connected");
      thingsBoard.loop();
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
  }
}

void sendDataToTelemetry(void *pvParameters){
  (void)pvParameters;
  while (true){
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void readDHT20(void *pvParameters)
{
  while(1){
    dht20.read();
    temperature = dht20.getTemperature();
    humidity = dht20.getHumidity();

    if (isnan(temperature) || isnan(humidity)){
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");

    bool success = thingsBoard.sendTelemetryData("temperature", temperature) && thingsBoard.sendTelemetryData("humidity", humidity);
    if (success)
      Serial.println("Send data to CoreIoT successfully");
    Serial.println();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for serial monitor to open
  delay(10000);
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("Initializing DHT20...");

  // Initialize DHT20
  if (!dht20.begin())
  {
    Serial.println("Could not find a valid DHT20 sensor!");
    while (1)
      ;
  }

  setupRelays();
  
  xTaskCreate(manageWifi, "ManageWifiTask", 4096, NULL, 1, NULL);
  xTaskCreate(connectToCoreIoT, "ConnectToCoreIoTTask", 4096, NULL, 1, NULL);
  xTaskCreate(readDHT20, "ReadDHT20Task", 4096, NULL, 2, NULL); // Higher priority
}

void loop()
{
  // Nothing to do here, FreeRTOS tasks handle the work
}






