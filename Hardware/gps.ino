#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets_sam.h"
#include <math.h>

#define GPS_SERIAL_RX D6
#define GPS_SERIAL_TX D5
#define TIME_ZONE 5.5
#define BUTTON_PIN D3
#define BUZZER_PIN D8

SoftwareSerial gpsSerial(GPS_SERIAL_RX, GPS_SERIAL_TX);
TinyGPSPlus gps;

time_t now;
Adafruit_MPU6050 mpu;

#define LCD_I2C_ADDRESS 0x27
LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 16, 2);

const char *AWS_IOT_TOPIC = "esp8266/sureway/pub";
const char *ESP32_SUB_TOPIC = "esp32/sub";
const char *ESP8266_ALERT_TOPIC = "esp8266/alert";
WiFiClientSecure net;
BearSSL::X509List cert(cacert);
BearSSL::X509List client_crt(client_cert);
BearSSL::PrivateKey key(privkey);
PubSubClient client(net);

StaticJsonDocument<400> doc;

// Define a flag to indicate whether an impact has been detected
bool impactDetected = false;

// Define a variable to store the time when the impact was detected
unsigned long impactTime = 0;

bool buzzerState = HIGH; // Initial state of the buzzer
bool buttonPressed = false; // Flag to track button press

TinyGPSLocation previousLocation;  // Declare previousLocation variable

void NTPConnect(void) {
  Serial.print("Setting time using SNTP");
  configTime(TIME_ZONE * 3600, 0 * 3600, "pool.ntp.org", "time.nist.gov");
  now = time(nullptr);
  while (now < 1510592825) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("done!");
}

void connectAWS() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("     SYSTEM     ");

  lcd.setCursor(0, 1);
  lcd.print("  INITIALIZING  ");

  delay(3000);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println(String("Attempting to connect to SSID: ") + String(WIFI_SSID));

  unsigned long connectStartMillis = millis();

  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - connectStartMillis > 15000) {  // Timeout after 15 seconds
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CONNECTION  LOST");
      Serial.println("WiFi Connection Timeout");
      delay(3000);
      ESP.restart();  // Reset the ESP8266
    }
    Serial.print(".");
    delay(1000);
  }

  NTPConnect();

  net.setTrustAnchors(&cert);
  net.setClientRSACert(&client_crt, &key);

  client.setServer(MQTT_HOST, 8883);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("...CONNECTING...");
  lcd.setCursor(0, 1);
  lcd.print(">>>>>>>><<<<<<<<");

  unsigned long awsConnectStartMillis = millis();

  while (!client.connect(THINGNAME)) {
    if (millis() - awsConnectStartMillis > 15000) {  // Timeout after 15 seconds
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("CONNECTION  LOST");
      Serial.println("AWS IoT Connection Timeout");
      delay(3000);
      ESP.restart();  // Reset the ESP8266
    }
    Serial.print(".");
    delay(1000);
  }

  if (client.connected()) {
    Serial.println("AWS IoT Connected!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("<<<<CONNECTED>>>>");
    lcd.setCursor(0, 1);
    lcd.print(">>>>>>>><<<<<<<<");
    delay(2000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(">>>>SURE WAY<<<<");
    lcd.setCursor(0, 1);
    lcd.print(">>>><<<<>>>><<<<");
  } else {
    Serial.println("AWS IoT Timeout!");
  }
}

void controlBuzzer(int state) {
  // Update the state of the buzzer
  digitalWrite(BUZZER_PIN, state);
  buzzerState = state;
}

void publishData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  struct tm *timeinfo;
  time(&now);
  timeinfo = localtime(&now);

  float speedkmph = gps.speed.kmph();  // Corrected variable name

  Serial.print("Speed: ");
  Serial.print(speedkmph);
  Serial.println(" kmph");

  // Create a local copy of the JSON document
  StaticJsonDocument<400> localDoc = doc;
  localDoc["time"] = String(timeinfo->tm_hour) + ":" + String(timeinfo->tm_min) + ":" + String(timeinfo->tm_sec);
  doc["angularX"] = g.gyro.x;
  doc["angularY"] = g.gyro.y;
  doc["angularZ"] = g.gyro.z;
  localDoc["latitude"] = gps.location.isValid() ? gps.location.lat() : 0.0;
  localDoc["longitude"] = gps.location.isValid() ? gps.location.lng() : 0.0;
  localDoc["speed"] = speedkmph;
  localDoc["thingName"] = "SN0013";

  char jsonBuffer[512];
  serializeJson(localDoc, jsonBuffer);

  Serial.println("Before MQTT publish");
  if (client.publish(AWS_IOT_TOPIC, jsonBuffer)) {
    Serial.println("Publish success");
  } else {
    Serial.println("Publish failed");
  }
  Serial.println("After MQTT publish");
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  Wire.begin();
  pinMode(BUTTON_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.backlight();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  delay(2000);

  lcd.setCursor(0, 0);
  lcd.print(">>>>SURE WAY<<<<");
  lcd.setCursor(0, 1);
  lcd.print(">>>><<<<>>>><<<<");

  connectAWS();
}

void loop() {
  now = time(nullptr);

  // Check if MQTT client is connected, if not, attempt to reconnect
  if (!client.connected()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONNECTION  LOST");
    lcd.setCursor(0, 1);
    lcd.print("..RECONNECTING..");

    unsigned long reconnectStartMillis = millis();
    while (!client.connected()) {
      if (millis() - reconnectStartMillis > 15000) {  // Timeout after 15 seconds
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("RECONNECT FAILED");
        lcd.setCursor(0, 1);
        lcd.print(">>>>>>>><<<<<<<<");
        Serial.println("MQTT Reconnection Timeout");
        delay(3000);
        ESP.restart();  // Reset the ESP8266
      }

      // Attempt to reconnect
      Serial.println("Attempting MQTT Reconnection...");
      if (client.connect(THINGNAME)) {
        Serial.println("Reconnected to AWS IoT!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("..RECONNECTED..");
        lcd.setCursor(0, 1);
        lcd.print(">>>><<<<>>>><<<<");
        delay(2000);
      } else {
        Serial.print("Failed to reconnect. Retrying in 1 second...");
        delay(1000);
      }
    }
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Accelerometer Data - X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.println(a.acceleration.z);

  // Store the previous location
  previousLocation = gps.location;

  float impactThreshold = 10.0;
  if (a.acceleration.x > impactThreshold || a.acceleration.y > impactThreshold || a.acceleration.z > impactThreshold) {
    // If an impact is detected and the impact flag is not set
    if (!impactDetected) {
      // Set the impact flag and record the time
      impactDetected = true;
      impactTime = millis();
      Serial.println("Impact detected!");

      // Create a JSON object
      StaticJsonDocument<100> jsonDoc;

      // Set the thingName in the JSON object
      jsonDoc["thingName"] = "SN0013";
      // Add other fields if needed
      jsonDoc["message"] = "Impact detected!";

      // Serialize the JSON object to a string
      char jsonStr[100];
      serializeJson(jsonDoc, jsonStr);

      // Publish the JSON message
      if (client.publish(ESP32_SUB_TOPIC, jsonStr, 0)) {
        Serial.println("Impact message published");
      } else {
        Serial.println("Failed to publish impact message");
      }
    }
  }

  // Check if impact alarm is active
  if (impactDetected) {
    // Calculate the time elapsed since the impact
    unsigned long elapsedTime = millis() - impactTime;

    lcd.clear();
    // Turn on the buzzer if the timer is running
    if (elapsedTime < 20000 && !buttonPressed) {
      // Display the remaining time on the LCD
      lcd.setCursor(0, 0);
      lcd.print("IMPACT  DETECTED");
      lcd.setCursor(0, 1);
      lcd.print("TIME LEFT: " + String(20 - elapsedTime / 1000));
      controlBuzzer(HIGH); // Turn on the buzzer

      if (digitalRead(BUTTON_PIN) == LOW) {
        // If D3 pin is low (button pressed), cancel the impact timer
        impactDetected = false;
        controlBuzzer(LOW); // Turn off the buzzer
        buttonPressed = true;

        // Display "Alarm off" on the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("...ALARM<>OFF...");
        delay(3000);

        // Return to normal state and display "SureWay"
        lcd.clear();
        lcd.print(">>>>SURE WAY<<<<");
        lcd.setCursor(0, 1);
        lcd.print(">>>><<<<>>>><<<<");
        buttonPressed = false;
      }
    } else {
      // If 20 seconds have passed or the button is pressed, reset the impact flag, turn off the buzzer, and clear the LCD
      impactDetected = false;
      controlBuzzer(LOW); // Turn off the buzzer

      // Display relevant message on the LCD
      lcd.clear();
      lcd.setCursor(0, 0);

      if (elapsedTime >= 20000) {
        // Timer expired, show alert message
        lcd.setCursor(0, 0);
        lcd.print("...ALERT SENT...");
        lcd.setCursor(0, 1);
        lcd.print(">>>><<<<>>>><<<<");
        delay(4000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(">>>>SURE WAY<<<<");
        lcd.setCursor(0, 1);
        lcd.print(">>>><<<<>>>><<<<");

        // Prepare the alert message
        StaticJsonDocument<100> alertDoc;
        alertDoc["thingName"] = "SN0013";
        alertDoc["message"] = "Alert";

        char alertBuffer[200];
        serializeJson(alertDoc, alertBuffer);

        // Publish the alert message to the specified topic
        if (client.publish(ESP8266_ALERT_TOPIC, alertBuffer)) {
          Serial.println("Alert message published");
        } else {
          Serial.println("Failed to publish alert message");
        }

        // Display "SureWay" after processing the alert or button press
        lcd.setCursor(0, 0);
        lcd.print(">>>>SURE WAY<<<<");
        lcd.setCursor(0, 1);
        lcd.print(">>>><<<<>>>><<<<");
      }
    }
  }

  unsigned long startMillis = millis();
  while (millis() - startMillis < 100) { // Reduced delay to 100 milliseconds
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (gps.location.isValid()) {
          float speedkmph = gps.speed.kmph();

          Serial.print("Speed: ");
          Serial.print(speedkmph);
          Serial.println(" kmph");

          // Include speed in the JSON document
          doc["speed"] = speedkmph;
        }
      }
    }
  }

  // Execute MQTT client loop regardless of the connection status
  client.loop();

  if (client.connected()) {
    publishData();
  }
}
