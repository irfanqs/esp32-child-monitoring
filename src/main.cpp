#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Fall_Detection_Model_inferencing.h>

// ESP_ID
#define DEVICE_ID "nino_001"

// PIN DEFINITION
#define IMU_SDA 8
#define IMU_SCL 9
#define VIBRATION_PIN 3

// MQTT Configuration (HiveMQ Cloud)
const char* mqtt_server    = "c634c8c1cdf54935970298ed1add5ed7.s1.eu.hivemq.cloud";
const int   mqtt_port      = 8883;
const char* mqtt_client_id = "ESP32_Main_Controller";
const char* mqtt_user      = "irfanqs";
const char* mqtt_pass      = "Irfan123";

// MQTT Topics
const String TOPIC_STATUS  = String("nino/") + String(DEVICE_ID) + "/status";
const String TOPIC_COMMAND = String("nino/") + String(DEVICE_ID) + "/command";

// WiFi & MQTT Clients
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// =====================
// Hardcoded WiFi config
#define WIFI_SSID "J2 Prime Turbo"
#define WIFI_PASSWORD "Qobus123"
// =====================
// Fall Detection Variables
Adafruit_MPU6050 myIMU;
float ax, ay, az;
int fallCounter = 0;
unsigned long lastVIBRATIONTime = 0;
const unsigned long VIBRATION_COOLDOWN = 3000; // 3 detik cooldown untuk debugging

String inferencingResult = "";
#define FREQUENCY_HZ EI_CLASSIFIER_FREQUENCY
#define INTERVAL_MS (1000 / (FREQUENCY_HZ + 1))
float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
size_t feature_ix = 0;

// Faint Detection Variables
#define FAINT_CHECK_DURATION_MS 30000 // 30 detik untuk deteksi pingsan
volatile int faintWatch = 0;          // Global variable to trigger faint check

// Parent Alert Variables
unsigned long lastParentAlertTime = 0;
const unsigned long PARENT_ALERT_COOLDOWN = 60000; // Cooldown 60 detik untuk parent alert

// MQTT Reconnection
unsigned long lastMqttReconnectAttempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL = 5000;

// Forward declarations
void playFallAlert();
void playParentAlert();
void sendStatus(String kondisi);

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char payloadStr[length + 1];
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0';

  Serial.printf("📥 MQTT message received on topic: %s\n", topic);
  Serial.printf("📥 Payload: %s\n", payloadStr);

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payloadStr);

  if (error) {
    Serial.printf("❌ JSON parse error: %s\n", error.c_str());
    return;
  }

  const char* posisiOrtu = doc["posisi_ortu_dekat"];

  if (posisiOrtu && strcmp(posisiOrtu, "ya") == 0) {
    if (millis() - lastParentAlertTime < PARENT_ALERT_COOLDOWN) {
      Serial.printf("⏰ Parent alert masih dalam cooldown (%lu ms remaining)\n",
                    PARENT_ALERT_COOLDOWN - (millis() - lastParentAlertTime));
    } else {
      Serial.println("🚨 Parent nearby detected! Playing parent alert...");
      playParentAlert();
      lastParentAlertTime = millis();
    }
  }
}

bool mqttConnect() {
  Serial.printf("📡 Connecting to MQTT: %s:%d... ", mqtt_server, mqtt_port);

  espClient.setInsecure();

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  if (mqttClient.connect(mqtt_client_id, mqtt_user, mqtt_pass)) {
    Serial.println("✅ Connected to HiveMQ Cloud!");
    mqttClient.subscribe(TOPIC_COMMAND.c_str());
    Serial.printf("📡 Subscribed to: %s\n", TOPIC_COMMAND.c_str());
    return true;
  } else {
    Serial.printf("❌ Failed (rc=%d)\n", mqttClient.state());
    return false;
  }
}

// I2C Scanner function
void scanI2C() {
  Serial.println("🔍 Scanning I2C devices...");
  byte error, address;
  int nDevices = 0;

  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.printf("✅ I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error == 4) {
      Serial.printf("❓ Unknown error at address 0x%02X\n", address);
    }
  }

  if (nDevices == 0) {
    Serial.println("❌ No I2C devices found!");
    Serial.println("🔧 Check wiring:");
    Serial.println("   - VCC to 3.3V");
    Serial.println("   - GND to GND");
    Serial.println("   - SDA to pin 8");
    Serial.println("   - SCL to pin 9");
  } else {
    Serial.printf("✅ Found %d I2C device(s)\n", nDevices);
  }
  Serial.println("");
}

// VIBRATION function untuk VIBRATION aktif (LOW = diam, HIGH = bunyi)
void playFallAlert() {
  Serial.println("DEBUG: playFallAlert() called");

  // Cek cooldown period
  if (millis() - lastVIBRATIONTime < VIBRATION_COOLDOWN) {
    Serial.printf("⏰ VIBRATION masih dalam cooldown period (%lu ms remaining)\n",
                  VIBRATION_COOLDOWN - (millis() - lastVIBRATIONTime));
    return;
  }

  Serial.println("🔊 === FALL ALERT ACTIVATED ===");

  // Bunyi 2 kali untuk fall alert
  for (int i = 0; i < 2; i++) {
    digitalWrite(VIBRATION_PIN, HIGH);   // Bunyi ON
    delay(400);
    digitalWrite(VIBRATION_PIN, LOW);  // Bunyi OFF
    if (i < 1) delay(200); // Jeda antar bunyi
  }

  lastVIBRATIONTime = millis();
  Serial.println("🔊 === VIBRATION ALERT COMPLETED ===");
}

void playParentAlert() {
  Serial.println("🔊 === PARENT NEARBY ALERT ===");

  // Bunyi 10 kali untuk parent alert
  for (int i = 0; i < 10; i++) {
    digitalWrite(VIBRATION_PIN, HIGH);   // Bunyi ON
    delay(200);
    digitalWrite(VIBRATION_PIN, LOW);  // Bunyi OFF
    if (i < 9) delay(150); // Jeda antar bunyi
  }

  Serial.println("🔊 === PARENT ALERT COMPLETED ===");
}

void sendStatus(String kondisi) {
  if (!mqttClient.connected()) {
    Serial.println("❌ MQTT not connected, cannot send status");
    return;
  }

  StaticJsonDocument<192> doc;
  doc["kondisi"] = kondisi;
  doc["device_id"] = DEVICE_ID;

  char buffer[192];
  size_t n = serializeJson(doc, buffer);

  bool result = mqttClient.publish(TOPIC_STATUS.c_str(), buffer);

  if (result) {
    Serial.printf("📤 Published to %s: %s\n", TOPIC_STATUS.c_str(), buffer);
  } else {
    Serial.printf("❌ Failed to publish to %s\n", TOPIC_STATUS.c_str());
  }
}

void faintCheck(void *pvParameters) {
  const float JERK_THRESHOLD = 35;
  const int SAMPLE_INTERVAL_MS = 100;
  unsigned long startTime = millis();

  float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;

  Serial.println("⏱️ Faint check started - monitoring for 30 seconds...");

  while (1) {
    float acceleration_g_x = ax;
    float acceleration_g_y = ay;
    float acceleration_g_z = az;

    if (prevAccX != 0.0 || prevAccY != 0.0 || prevAccZ != 0.0) {
      float jerkX = (acceleration_g_x - prevAccX) / (SAMPLE_INTERVAL_MS / 1000.0);
      float jerkY = (acceleration_g_y - prevAccY) / (SAMPLE_INTERVAL_MS / 1000.0);
      float jerkZ = (acceleration_g_z - prevAccZ) / (SAMPLE_INTERVAL_MS / 1000.0);

      float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

      if (jerkMagnitude > JERK_THRESHOLD) {
        Serial.printf("✅ Movement detected (jerk: %.2f) - No fainting\n", jerkMagnitude);
        faintWatch = 0;
        vTaskDelete(NULL);
      }
    }

    prevAccX = acceleration_g_x;
    prevAccY = acceleration_g_y;
    prevAccZ = acceleration_g_z;

    if (millis() - startTime >= FAINT_CHECK_DURATION_MS) {
      Serial.println("🚨 No significant movement for 30 seconds - FAINTING DETECTED!");
      sendStatus("pingsan");
      faintWatch = 0;
      vTaskDelete(NULL);
    }

    vTaskDelay(SAMPLE_INTERVAL_MS / portTICK_PERIOD_MS);
  }
}

void monitorFaintWatch(void *pvParameters) {
  while (1) {
    if (faintWatch == 1) {
      Serial.println("⏱️ faintWatch activated. Waiting 6 seconds before monitoring...");
      vTaskDelay(6000 / portTICK_PERIOD_MS);

      xTaskCreatePinnedToCore(
        faintCheck,
        "FaintCheck",
        4096,
        NULL,
        1,
        NULL,
        1);

      faintWatch = 0;
    }

    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void imuTask(void *pvParameters)
{
  unsigned long last_interval_ms = 0;
  bool isWarmedUp = false;
  unsigned long warmUpStartTime = millis();
  const unsigned long WARMUP_DURATION = 5000; // 5 detik warm-up

  for (;;)
  {
    if (!isWarmedUp) {
      if (millis() - warmUpStartTime < WARMUP_DURATION) {
        if (millis() - last_interval_ms >= INTERVAL_MS) {
          last_interval_ms = millis();

          sensors_event_t a, g, temp;
          myIMU.getEvent(&a, &g, &temp);

          ax = a.acceleration.x;
          ay = a.acceleration.y;
          az = a.acceleration.z;

          features[feature_ix++] = -ay;
          features[feature_ix++] = ax;
          features[feature_ix++] = az;

          if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            feature_ix = 0;
          }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
        continue;
      } else {
        isWarmedUp = true;
        feature_ix = 0;
        Serial.println("🔥 IMU WARM-UP COMPLETED - Fall detection active!");
      }
    }

    if (millis() - last_interval_ms >= INTERVAL_MS)
    {
      last_interval_ms = millis();

      sensors_event_t a, g, temp;
      myIMU.getEvent(&a, &g, &temp);

      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;

      features[feature_ix++] = -ay;
      features[feature_ix++] = ax;
      features[feature_ix++] = az;

      if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
      {
        ei_impulse_result_t result;
        signal_t signal;
        numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != 0) {
          Serial.println("Error running classifier");
          feature_ix = 0;
          continue;
        }

        float maxValue = 0.0f;
        int maxIndex = 0;

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
          if (result.classification[ix].value > maxValue)
          {
            maxValue = result.classification[ix].value;
            maxIndex = ix;
          }
        }

        inferencingResult = result.classification[maxIndex].label;

        if (inferencingResult == "Fall" && maxValue > 0.7)
        {
          fallCounter++;
          Serial.printf("🚨 Fall detected! Confidence: %.2f%%, Counter: %d\n", maxValue * 100, fallCounter);

          if (fallCounter >= 1)
          {
            Serial.println("🔥 CONFIRMED FALL - Triggering VIBRATION and sending alert!");
            playFallAlert();
            sendStatus("terjatuh");
            faintWatch = 1;
            fallCounter = 0;
          }
        }
        else
        {
          if (fallCounter > 0) {
            Serial.printf("⚠️ Resetting fall counter - Current: %s (%.1f%%)\n",
                         inferencingResult.c_str(), maxValue * 100);
          }
          fallCounter = 0;
        }
        feature_ix = 0;
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n========================================");
  Serial.println("  FALL DETECTION + HIVEMQ MQTT SYSTEM");
  Serial.println("========================================");
  Serial.printf("Device ID: %s\n", DEVICE_ID);
  Serial.printf("MQTT Status Topic: %s\n", TOPIC_STATUS.c_str());
  Serial.printf("MQTT Command Topic: %s\n", TOPIC_COMMAND.c_str());

  // Initialize VIBRATION pin
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);
  Serial.println("Step 1: ✅ VIBRATION initialized");

  // Test VIBRATION
  Serial.println("Step 2: Testing VIBRATION...");
  digitalWrite(VIBRATION_PIN, HIGH);
  delay(300);
  digitalWrite(VIBRATION_PIN, LOW);
  Serial.println("Step 2: ✅ VIBRATION test completed");

  // Initialize WiFi via WiFiManager
  Serial.println("Step 3: Starting WiFi (hardcoded only)...");

  if (strlen(WIFI_SSID) == 0) {
    Serial.println("Step 3: ❌ WIFI_SSID kosong - tidak ada mekanisme konfigurasi captive portal karena WiFiManager dihapus.");
    Serial.println("Silakan isi WIFI_SSID dan WIFI_PASSWORD di src/main.cpp atau gunakan metode lain untuk memasukkan kredensial.");
    delay(5000);
    ESP.restart();
    return;
  }

  Serial.printf("Step 3: Trying hardcoded WiFi SSID: %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 30000) {
    Serial.print(".");
    delay(500);
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nStep 3: ✅ WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nStep 3: ❌ Hardcoded WiFi failed to connect within timeout. Restarting...");
    delay(2000);
    ESP.restart();
    return;
  }

  // Initialize MQTT
  Serial.println("Step 4: Connecting to HiveMQ Cloud...");
  mqttClient.setCallback(mqttCallback);
  if (mqttConnect()) {
    Serial.println("Step 4: ✅ MQTT connected!");
  } else {
    Serial.printf("Step 4: ⚠️ MQTT connection failed (rc=%d), will retry in loop\n", mqttClient.state());
  }

  // Initialize I2C
  Wire.begin(IMU_SDA, IMU_SCL);
  Serial.printf("Step 5: ✅ I2C initialized (SDA:%d, SCL:%d)\n", IMU_SDA, IMU_SCL);

  // Scan for I2C devices
  scanI2C();

  // Initialize MPU6050
  Serial.print("Step 6: Initializing MPU6050... ");
  if (!myIMU.begin()) {
    Serial.println("❌ FAILED!");
    Serial.println("🔧 Troubleshooting steps:");
    Serial.println("1. Check if MPU6050 is detected in I2C scan above");
    Serial.println("2. Verify wiring connections");
    Serial.println("3. Check power supply (3.3V)");
    Serial.println("4. Try different I2C pins if needed");
    Serial.println("⚠️ Continuing without MPU6050 - Fall detection disabled");

    delay(2000);
  } else {
    Serial.println("✅ SUCCESS!");

    myIMU.setFilterBandwidth(MPU6050_BAND_10_HZ);
    myIMU.setAccelerometerRange(MPU6050_RANGE_8_G);
    myIMU.setGyroRange(MPU6050_RANGE_1000_DEG);
    Serial.println("Step 7: ✅ MPU6050 configured");
  }

  // Create tasks
  xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(monitorFaintWatch, "FaintWatchTask", 2048, NULL, 1, NULL, 1);

  Serial.println("Step 8: ✅ Tasks created");

  // Kirim status start
  Serial.println("Step 9: Sending 'start' status via MQTT...");
  sendStatus("start");
  delay(1000);
  Serial.println("Step 9: ✅ Start status sent");

  Serial.println("========================================");
  Serial.println("✅ SYSTEM READY!");
  Serial.println("🔊 VIBRATION: Ready (LOW=diam, HIGH=bunyi)");
  Serial.print("📡 WiFi: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.print("☁️ MQTT: ");
  Serial.println(mqttClient.connected() ? "Connected" : "Disconnected");
  Serial.println("📊 IMU: Ready");
  Serial.println("🚨 Fall Detection: Active");
  Serial.println("😴 Faint Detection: Active");
  Serial.printf("📡 MQTT Server: %s:%d\n", mqtt_server, mqtt_port);
  Serial.printf("📤 Status Topic: %s\n", TOPIC_STATUS.c_str());
  Serial.printf("📥 Command Topic: %s\n", TOPIC_COMMAND.c_str());
  Serial.println("========================================\n");
}

void loop()
{
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > MQTT_RECONNECT_INTERVAL) {
      lastMqttReconnectAttempt = now;
      Serial.println("⚠️ MQTT disconnected, attempting reconnect...");
      if (mqttConnect()) {
        Serial.println("✅ MQTT reconnected!");
      }
    }
  } else {
    mqttClient.loop();
  }

  // Status monitoring setiap 5 detik
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 5000) {
    lastStatusUpdate = millis();

    sensors_event_t a, g, temp;
    myIMU.getEvent(&a, &g, &temp);

    Serial.println("========== SYSTEM STATUS ==========");
    Serial.printf("⏱️  Uptime: %lu seconds\n", millis()/1000);
    Serial.printf("📡 WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("☁️ MQTT: %s\n", mqttClient.connected() ? "Connected" : "Disconnected");
    Serial.printf("🎯 Prediction: %s, Fall counter: %d\n", inferencingResult.c_str(), fallCounter);
    Serial.printf("📊 IMU: X:%.2f Y:%.2f Z:%.2f m/s²\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("😴 Faint watch: %s\n", faintWatch ? "Active" : "Inactive");
    Serial.printf("🔊 VIBRATION: %s\n", digitalRead(VIBRATION_PIN) ? "Bunyi" : "Diam");
    Serial.println("===================================\n");
  }

  delay(100);
}
