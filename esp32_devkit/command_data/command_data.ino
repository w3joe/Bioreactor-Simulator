#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <TFT_eSPI.h> 
#include <ArduinoJson.h> 

// ================= 1. UCL Eduroam  =================
#define USE_EDUROAM 1
#define WIFI_USER "zcabwuh@ucl.ac.uk"
#define WIFI_SSID "eduroam"
#define WIFI_PASS "Me_52248412135792468"

#include "WiFi.h"

#if USE_EDUROAM
#include "esp_eap_client.h"
const char* ssid = "eduroam";
const char* user = WIFI_USER;
#else
const char* ssid = WIFI_SSID;
#endif

const char* pass = WIFI_PASS;

// ================= 2. HiveMQ Server  =================
const char* mqtt_server = "444f9f1a6d4748f7a238f6d0a8068877.s1.eu.hivemq.cloud";
const int mqtt_port     = 8883;

// Access the Broker
const char* mqtt_user = "Engineering-Challenges_Group5_Broker"; 
const char* mqtt_pass = "HelloWorld:0";

// ================= 3. Topic definition =================
const char* topic_pub = "ucl/ec2/group5/ttgo/data";    
const char* topic_sub = "ucl/ec2/group5/ttgo/command"; 

WiFiClientSecure espClient; 
PubSubClient client(espClient);
TFT_eSPI tft = TFT_eSPI();

// global variant
int current_rpm = 0;
float current_temp = 20.0;
float current_ph = 7.0;

// ... (print_wifi_info) ...
void print_wifi_info () {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected to WiFi network: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Gateway address: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Subnet mask: ");
    Serial.println(WiFi.subnetMask());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
  } else {
    Serial.println("WiFi not connected");
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
  }
}

// ... (wifi_connect) ...
void wifi_connect ( float timeout = 15 ) {
  unsigned long deadline;
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
#if USE_EDUROAM
  Serial.printf("Connecting to eduroam as user %s\n", user);
  esp_eap_client_set_identity((uint8_t *)user, strlen(user));
  esp_eap_client_set_username((uint8_t *)user, strlen(user));
  esp_eap_client_set_password((uint8_t *)pass, strlen(pass));
  esp_wifi_sta_enterprise_enable();
  WiFi.begin(ssid);
#else
  Serial.printf("Connecting to %s\n", ssid);
  WiFi.begin(ssid, pass);
#endif
  deadline = millis() + (unsigned long)(timeout * 1000);
  while ((WiFi.status() != WL_CONNECTED) && (millis() < deadline)) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  print_wifi_info();
}

// ... (reconnect) ...
void reconnect() {
  while (!client.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost! Reconnecting to Eduroam...");
      tft.fillScreen(TFT_BLACK);
      tft.setCursor(0,0);
      tft.println("WiFi Lost...");
      wifi_connect(60); 
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connecting MQTT...");
      String clientId = "TTGO-" + String(random(0xffff), HEX);
      if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
        Serial.println("Success!");
        client.subscribe(topic_sub);
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(0,0);
        tft.setTextColor(TFT_GREEN);
        tft.println("MQTT OK!");
      } else {
        Serial.print("Failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5s");
        delay(5000);
      }
    }
  }
}


// ... (callback) ...
int target_rpm = 0;
int target_temp = 20;
int target_pH = 0;

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("get command! [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  JsonDocument doc; 
  
  //json analysis
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print(F("Analysis failed: "));
    Serial.println(error.f_str());
    return;
  }
  

  // set stirring system
  if (doc.containsKey("target_rpm")) {
    target_rpm = doc["target_rpm"];

    //print on serial
    Serial.print("Set target_rpm: ");
    Serial.println(target_rpm);
    
    
    // show on the screen
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_YELLOW);
    tft.print("New RPM:\n");
    tft.setTextSize(3);
    tft.print(target_rpm);
    tft.println(" RPM");
  }

  // set temp
  if (doc.containsKey("target_temp")) {
    target_temp = doc["target_temp"];

    //print on serial
    Serial.print("Set target_temp: ");
    Serial.println(target_temp);
    
    
    // show on the screen
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_YELLOW);
    tft.print("New Temp:\n");
    tft.setTextSize(3);
    tft.print(target_temp);
    tft.println("degrees");
    
  }

  // set pH
  if (doc.containsKey("target_pH")) {
    target_pH = doc["target_pH"];

    //print on serial
    Serial.print("Set target_pH: ");
    Serial.println(target_pH);
    
    
    // show on the screen
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0);
    tft.setTextColor(TFT_YELLOW);
    tft.print("New pH:\n");
    tft.setTextSize(3);
    tft.print(target_pH);
    
  }
}

// ... (setup) ...
void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);

  wifi_connect();

  espClient.setInsecure();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}


void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static long lastMsg = 0;
  //every 5 seconds
  if (millis() - lastMsg > 5000) {
    lastMsg = millis();

    current_rpm = random(800, 1500); 
    
    current_temp = 20.0 + (random(0, 400) / 10.0);
    
    // 模拟 pH 值 (6.0 - 8.0)，保留一位小数
    current_ph = 6.0 + (random(0, 200) / 10.0);

    // send them with json
    JsonDocument doc;
    doc["rpm"] = current_rpm;
    doc["temp"] = current_temp;
    doc["ph"] = current_ph;

    // serialize it 
    char buffer[256];
    serializeJson(doc, buffer);

    client.publish(topic_pub, buffer);
    
    Serial.print("Sending JSON: ");
    Serial.println(buffer);
    
    tft.setCursor(0, 40);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.print("Sent Data...");
  }
}