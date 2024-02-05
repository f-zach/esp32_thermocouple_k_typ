#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>

typedef struct ESPnowMesssage
{
  int ID = 4;
  double DataReadings[16];
  int ControlData[8];
  bool ControlFlags[8];
} ESPnowMesssage;

ESPnowMesssage MessageRingBase;
ESPnowMesssage InBuffer;

uint8_t broadcastAddressMain[] = {0x98, 0xCD, 0xAC, 0x60, 0xDF, 0x28};
esp_now_peer_info MainStation;

long tLED;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  tLED = millis();
  digitalWrite(2, HIGH);
  Serial.println("Message sent!");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&InBuffer, incomingData, sizeof(InBuffer));
  esp_now_send(broadcastAddressMain, (uint8_t *)&MessageRingBase, sizeof(MessageRingBase));
}

//Create all MAX31856 objects with their corresponding CS-Pin
Adafruit_MAX31856 TypeT1(27);
Adafruit_MAX31856 TypeT2(25);
Adafruit_MAX31856 TypeT3(32);
Adafruit_MAX31856 TypeT4(12);

long tStartMeasurement = 0;

float KTypeTemp[4];

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  SPI.begin();

  pinMode(2, OUTPUT);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(WIFI_PS_NONE);
  Serial.println("MAC-Adress: " + WiFi.macAddress());

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);

  memcpy(MainStation.peer_addr, broadcastAddressMain, 6);
  MainStation.channel = WiFi.channel();
  MainStation.encrypt = false;

  if (esp_now_add_peer(&MainStation) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

 // Initialize the MAX31856 modules
  TypeT1.begin();
  TypeT1.setThermocoupleType(MAX31856_TCTYPE_K);
  TypeT1.setConversionMode(MAX31856_ONESHOT_NOWAIT);

  TypeT2.begin();
  TypeT2.setThermocoupleType(MAX31856_TCTYPE_T);
  TypeT2.setConversionMode(MAX31856_ONESHOT_NOWAIT);

  TypeT3.begin();
  TypeT3.setThermocoupleType(MAX31856_TCTYPE_T);
  TypeT3.setConversionMode(MAX31856_ONESHOT_NOWAIT);

  TypeT4.begin();
  TypeT4.setThermocoupleType(MAX31856_TCTYPE_T);
  TypeT4.setConversionMode(MAX31856_ONESHOT_NOWAIT);

  // Trigger first oneshot measurement
  TypeT1.triggerOneShot();
  TypeT2.triggerOneShot();
  TypeT3.triggerOneShot();
  TypeT4.triggerOneShot();

  tStartMeasurement = millis();
}

void loop()
{
  // Read temperature measurements and start another oneshot measurement
  if (millis() - tStartMeasurement >= 200)
  {
    MessageRingBase.DataReadings[0] = TypeT1.readThermocoupleTemperature();
    MessageRingBase.DataReadings[1] = TypeT2.readThermocoupleTemperature();
    MessageRingBase.DataReadings[2] = TypeT3.readThermocoupleTemperature();
    MessageRingBase.DataReadings[3] = TypeT4.readThermocoupleTemperature();

    TypeT1.triggerOneShot();
    TypeT2.triggerOneShot();
    TypeT3.triggerOneShot();
    TypeT4.triggerOneShot();

    tStartMeasurement = millis();

    for(int i = 0; i < 4; i++)
    {
      Serial.print(String(MessageRingBase.DataReadings[i])+ "\t");
    }
    Serial.println();
  }

  if (millis() - tLED >= 33)
  {
    digitalWrite(2, LOW);
  }
}