#include <Wire.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Fall_Detection_Model_inferencing.h>

// PIN DEFINITION
#define IMU_SDA 8
#define IMU_SCL 9
#define VIBRATION_PIN 3

// Antares Configuration
#define ANTARES_ACCESS_KEY "8183f9d73fe936db:40ab0de1a828f251"
#define ANTARES_APPLICATION "testbeneran"
#define ANTARES_DEVICE "esp32"
#define ANTARES_URL "https://platform.antares.id:8443/~/antares-cse/antares-id/" ANTARES_APPLICATION "/" ANTARES_DEVICE

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
unsigned long lastAntaresCheck = 0;
const unsigned long ANTARES_CHECK_INTERVAL = 10000; // Check setiap 10 detik

// Parent Alert Variables
String lastProcessedData = "";         // Menyimpan data terakhir yang sudah diproses
unsigned long lastParentAlertTime = 0; // Timestamp alert terakhir
const unsigned long PARENT_ALERT_COOLDOWN = 60000; // Cooldown 60 detik untuk parent alert

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

// VIBRATION function untuk VIBRATION aktif (LOW = bunyi, HIGH = diam)
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
    digitalWrite(VIBRATION_PIN, LOW);   // Bunyi ON
    delay(400);
    digitalWrite(VIBRATION_PIN, HIGH);  // Bunyi OFF
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

void sendToAntares(String kondisi) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("❌ WiFi not connected, cannot send to Antares");
    return;
  }

  HTTPClient http;
  http.begin(ANTARES_URL);
  
  // Header sesuai format yang berhasil
  http.addHeader("X-M2M-Origin", ANTARES_ACCESS_KEY);
  http.addHeader("Content-Type", "application/json;ty=4");  // Ubah dari ty=3 ke ty=4
  http.addHeader("Accept", "application/json");

  // Payload JSON sesuai format yang berhasil
  String payload = "{\"m2m:cin\": {\"con\": \"{\\\"kondisi\\\":\\\"" + kondisi + "\\\"}\"}}";
  
  Serial.printf("📤 Sending to Antares: %s\n", payload.c_str());
  
  int httpResponseCode = http.POST(payload);
  
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.printf("✅ Antares response: %d - %s\n", httpResponseCode, response.c_str());
  } else {
    Serial.printf("❌ Antares error: %d\n", httpResponseCode);
  }
  
  http.end();
}

void checkAntaresDownlink() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  HTTPClient http;
  // URL untuk mendapatkan data terbaru (latest data)
  String downlinkURL = String(ANTARES_URL) + "/la";
  http.begin(downlinkURL);
  
  // Header untuk GET request
  http.addHeader("X-M2M-Origin", ANTARES_ACCESS_KEY);
  http.addHeader("Accept", "application/json");
  
  int httpResponseCode = http.GET();
  
  if (httpResponseCode == 200) {
    String payload = http.getString();
    
    // Cek apakah payload sama dengan yang terakhir diproses
    if (payload == lastProcessedData) {
      Serial.println("📥 Same data as before, skipping processing");
      http.end();
      return;
    }
    
    Serial.printf("📥 Antares downlink (new): %s\n", payload.c_str());
    
    // Parse JSON untuk cek posisi_ortu_dekat
    StaticJsonDocument<1000> doc;
    DeserializationError error = deserializeJson(doc, payload);
    
    if (!error) {
      // Cek berbagai path yang mungkin dalam response Antares
      String posisiOrtu = "";
      
      // Cek format response Antares yang berbeda-beda
      if (doc["m2m:cin"]["con"]["posisi_ortu_dekat"]) {
        posisiOrtu = doc["m2m:cin"]["con"]["posisi_ortu_dekat"].as<String>();
      } else if (doc["m2m:cin"]["con"].is<String>()) {
        String conString = doc["m2m:cin"]["con"].as<String>();
        // Parse jika con berisi JSON string
        StaticJsonDocument<200> conDoc;
        DeserializationError conError = deserializeJson(conDoc, conString);
        if (!conError && conDoc["posisi_ortu_dekat"]) {
          posisiOrtu = conDoc["posisi_ortu_dekat"].as<String>();
        }
      }
      
      if (posisiOrtu == "ya") {
        // Cek cooldown untuk parent alert
        if (millis() - lastParentAlertTime < PARENT_ALERT_COOLDOWN) {
          Serial.printf("⏰ Parent alert masih dalam cooldown (%lu ms remaining)\n", 
                        PARENT_ALERT_COOLDOWN - (millis() - lastParentAlertTime));
        } else {
          Serial.println("🚨 Parent nearby detected! Playing parent alert...");
          playParentAlert();
          lastParentAlertTime = millis();
        }
      }
      
      // Simpan payload sebagai data terakhir yang diproses
      lastProcessedData = payload;
    } else {
      Serial.printf("❌ JSON parsing error: %s\n", error.c_str());
    }
  } else {
    Serial.printf("❌ Downlink error: %d\n", httpResponseCode);
  }
  
  http.end();
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
      // Calculate jerk (change in acceleration over time)
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

    // Check if 30 seconds have passed
    if (millis() - startTime >= FAINT_CHECK_DURATION_MS) {
      Serial.println("🚨 No significant movement for 30 seconds - FAINTING DETECTED!");
      sendToAntares("pingsan");
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

      // Create the faintCheck task
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

  for (;;)
  {
    // Check if it's time to sample
    if (millis() - last_interval_ms >= INTERVAL_MS)
    {
      last_interval_ms = millis();

      // Read accelerometer data
      sensors_event_t a, g, temp;
      myIMU.getEvent(&a, &g, &temp);

      ax = a.acceleration.x;
      ay = a.acceleration.y;
      az = a.acceleration.z;

      // Add accelerometer data to features buffer
      features[feature_ix++] = -ay;
      features[feature_ix++] = ax;
      features[feature_ix++] = az;

      // Check if features buffer is full
      if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE)
      {
        ei_impulse_result_t result;

        // Create the signal for inference
        signal_t signal;
        numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);

        // Run inference
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res != 0) {
          Serial.println("Error running classifier");
          feature_ix = 0;
          continue;
        }

        // Find the label with the highest probability
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

        // Update the global variable with the highest-valued label
        inferencingResult = result.classification[maxIndex].label;
        
        // Fall detection dengan threshold yang lebih rendah untuk debugging
        if (inferencingResult == "Fall" && maxValue > 0.5) // Threshold 50%
        {
          fallCounter++;
          Serial.printf("🚨 Fall detected! Confidence: %.2f%%, Counter: %d\n", maxValue * 100, fallCounter);
          
          if (fallCounter >= 1)
          {
            Serial.println("🔥 CONFIRMED FALL - Triggering VIBRATION and sending alert!");
            playFallAlert();
            sendToAntares("terjatuh");
            faintWatch = 1; // Start faint monitoring
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
    // Allow other tasks to run
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n========================================");
  Serial.println("  FALL DETECTION + ANTARES SYSTEM");
  Serial.println("========================================");

  // Initialize VIBRATION pin
  pinMode(VIBRATION_PIN, OUTPUT);
  digitalWrite(VIBRATION_PIN, LOW);  // LOW = diam
  Serial.println("Step 1: ✅ VIBRATION initialized");
  
  // Test VIBRATION
  Serial.println("Step 2: Testing VIBRATION...");
  digitalWrite(VIBRATION_PIN, HIGH);
  delay(300);
  digitalWrite(VIBRATION_PIN, LOW);
  Serial.println("Step 2: ✅ VIBRATION test completed");

  // Initialize WiFi via WiFiManager
  Serial.println("Step 3: Starting WiFiManager...");
  WiFiManager wm;

  // Jika ingin reset konfigurasi WiFi (misal saat debugging), aktifkan baris ini
  // wm.resetSettings();

  if (!wm.autoConnect("ESP32C3_AP", "12345678")) {
    Serial.println("Step 3: ❌ Failed to connect, restarting...");
    ESP.restart();
    delay(1000);
  } else {
    Serial.printf("Step 3: ✅ WiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
  }

  // Initialize I2C
  Wire.begin(IMU_SDA, IMU_SCL);
  Serial.printf("Step 4: ✅ I2C initialized (SDA:%d, SCL:%d)\n", IMU_SDA, IMU_SCL);
  
  // Scan for I2C devices
  scanI2C();

  // Initialize MPU6050
  Serial.print("Step 5: Initializing MPU6050... ");
  if (!myIMU.begin()) {
    Serial.println("❌ FAILED!");
    Serial.println("🔧 Troubleshooting steps:");
    Serial.println("1. Check if MPU6050 is detected in I2C scan above");
    Serial.println("2. Verify wiring connections");
    Serial.println("3. Check power supply (3.3V)");
    Serial.println("4. Try different I2C pins if needed");
    Serial.println("⚠️ Continuing without MPU6050 - Fall detection disabled");
    
    // Continue without MPU6050 for debugging
    delay(2000);
  } else {
    Serial.println("✅ SUCCESS!");
    
    // Configure MPU6050
    myIMU.setFilterBandwidth(MPU6050_BAND_10_HZ);
    myIMU.setAccelerometerRange(MPU6050_RANGE_8_G);
    myIMU.setGyroRange(MPU6050_RANGE_1000_DEG);
    Serial.println("Step 6: ✅ MPU6050 configured");
  }

  // Create tasks
  xTaskCreatePinnedToCore(imuTask, "IMUTask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(monitorFaintWatch, "FaintWatchTask", 2048, NULL, 1, NULL, 1);
  
  Serial.println("Step 7: ✅ Tasks created");
  Serial.println("========================================");
  Serial.println("✅ SYSTEM READY!");
  Serial.println("🔊 VIBRATION: Ready (LOW=diam, HIGH=bunyi)");
  Serial.print("📡 WiFi: ");
  Serial.println(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.println("📊 IMU: Ready");
  Serial.println("🚨 Fall Detection: Active");
  Serial.println("😴 Faint Detection: Active");
  Serial.println("☁️ Antares Integration: Ready");
  Serial.printf("🌐 Antares URL: %s\n", ANTARES_URL);
  Serial.println("========================================\n");
}

void loop()
{
  // Status monitoring setiap 5 detik
  static unsigned long lastStatusUpdate = 0;
  if (millis() - lastStatusUpdate > 5000) {
    lastStatusUpdate = millis();
    
    sensors_event_t a, g, temp;
    myIMU.getEvent(&a, &g, &temp);
    
    Serial.println("========== SYSTEM STATUS ==========");
    Serial.printf("⏱️  Uptime: %lu seconds\n", millis()/1000);
    Serial.printf("📡 WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("🎯 Prediction: %s, Fall counter: %d\n", inferencingResult.c_str(), fallCounter);
    Serial.printf("📊 IMU: X:%.2f Y:%.2f Z:%.2f m/s²\n", a.acceleration.x, a.acceleration.y, a.acceleration.z);
    Serial.printf("😴 Faint watch: %s\n", faintWatch ? "Active" : "Inactive");
    Serial.printf("🔊 VIBRATION: %s\n", digitalRead(VIBRATION_PIN) ? "Diam" : "Bunyi");
    Serial.println("===================================\n");
  }
  
  // Check Antares downlink setiap 10 detik
  if (millis() - lastAntaresCheck > ANTARES_CHECK_INTERVAL) {
    lastAntaresCheck = millis();
    checkAntaresDownlink();
  }
  
  delay(1000);
}