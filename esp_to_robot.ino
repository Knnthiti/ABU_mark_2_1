#include <ABU_67.h>

#define I2C_DEV_ADDR 9

ABU_ROBOT esp_to_robot;

uint8_t status_ESPNOW = 0;

// #define ESPNOW
// #define I2C
#define Joy

// Callback function executed when data is received
void OnDataRecvESPNOW(const uint8_t *mac, const uint8_t *incomingData, int len) {
  status_ESPNOW = 1;
  esp_to_robot.set_PS2_0();
  memcpy(&esp_to_robot.Str_PS2, incomingData, sizeof(esp_to_robot.Str_PS2));
#ifdef Joy
  esp_to_robot.print_PS2();
#endif
}

void master_to_slave(int len) {
#ifdef I2C
  Serial.print("onReceive : ");
#endif
  while (0 < Wire.available()) {
    char c = Wire.read();   /* receive byte as a character */
#ifdef I2C
    Serial.print(c);       /* print the character */
#endif
  }
#ifdef I2C
  Serial.print(" | ");
#endif
}

void slave_to_master() {
#ifdef I2C
  Serial.println("onRequest");
#endif

  if (status_ESPNOW == 1) {
    digitalWrite(2, 1);
    Wire.write((byte *)&esp_to_robot.Str_PS2, sizeof(esp_to_robot.Str_PS2));
    status_ESPNOW = 0;
#ifdef ESPNOW
    Serial.println("ESPNOW rev OK");
#endif
  } else {
    digitalWrite(2, 0);
    esp_to_robot.set_PS2_0();
    Wire.write((byte *)&esp_to_robot.Str_PS2, sizeof(esp_to_robot.Str_PS2));
#ifdef ESPNOW
    Serial.println("T_T ESPNOW notrev");
#endif
  }
  
}
void setup() {
  // Set up Serial Monitor
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Set ESP32 as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initilize ESP-NOW
  if (esp_now_init() != ESP_OK) {
#ifdef ESPNOW
    Serial.println("Error initializing ESP-NOW");
#endif
    return;
  }

  // Register callback function
  esp_now_register_recv_cb(OnDataRecvESPNOW);

  // Set up Wire
  Serial.setDebugOutput(true);
  Wire.onReceive(master_to_slave);
  Wire.onRequest(slave_to_master);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

void loop() {
}