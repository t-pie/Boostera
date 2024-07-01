#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>

#define SERVICE_UUID               "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define TUNING_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define ODOMETER_CHARACTERISTIC_UUID "d78a0d6e-8fc8-4b57-a9ef-5c4c2550e1b8"
#define OTA_CHARACTERISTIC_UUID "f2d8b6be-4629-4a1b-ae16-5fdb43e410f8"

// Manufacturer data
uint8_t manufacturerData[5] = {'E', 'B', 'I', 'K', 'E'}; // Example data
a
const uint LOW_PIN = 3;
const uint PULSE_PIN = 1;
const uint SENSOR1_PIN = 2;
const uint SENSOR2_PIN = 4;

volatile uint32_t current_interval = 500;
volatile bool bike_stopped = true;
const uint32_t MIN_INTERVAL_MS = 350;
const uint32_t DEBOUNCE_TIME_MS = 150;
const uint32_t PULSE_WIDTH_MS = 30;
volatile uint32_t last_pulse_time = 0;
volatile bool new_pulse = false;
volatile uint32_t actual_pulse_count = 0;
volatile uint32_t spoofed_pulse_count = 0;

volatile bool tuning_state = false;
volatile bool odometer_corrections = true;

BLECharacteristic *tuningCharacteristic;
BLECharacteristic *odometerCharacteristic;
BLECharacteristic *otaCharacteristic;

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (pCharacteristic->getUUID().toString() == TUNING_CHARACTERISTIC_UUID) {
      tuning_state = value[0] == 1;
      Serial.printf("Tuning state changed to: %s\n", tuning_state ? "ON" : "OFF");
    } else if (pCharacteristic->getUUID().toString() == ODOMETER_CHARACTERISTIC_UUID) {
      odometer_corrections = value[0] == 1;
      Serial.printf("Odometer corrections changed to: %s\n", odometer_corrections ? "ON" : "OFF");
    }
  }
};

class OTACharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (Update.hasError()) {
      Update.printError(Serial);
    }

    if (!Update.isRunning()) {
      // Start update
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
        return;
      }
    }

    // Write firmware data to flash
    if (Update.write((uint8_t*)value.c_str(), value.length()) != value.length()) {
      Update.printError(Serial);
      return;
    }

    if (Update.end(true)) {
      Serial.println("OTA update finished!");
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(LOW_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(SENSOR1_PIN, INPUT);
  pinMode(SENSOR2_PIN, OUTPUT);

  digitalWrite(LOW_PIN, LOW);
  digitalWrite(PULSE_PIN, LOW);
  pinMode(SENSOR1_PIN, INPUT_PULLDOWN);
  digitalWrite(SENSOR2_PIN, HIGH);

  attachInterrupt(digitalPinToInterrupt(SENSOR1_PIN), gpio_callback, RISING);

  BLEDevice::init("ESP32_BLE");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  tuningCharacteristic = pService->createCharacteristic(
      TUNING_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  tuningCharacteristic->setCallbacks(new MyCallbacks());
  tuningCharacteristic->setValue((uint8_t*)&tuning_state, 1);

  odometerCharacteristic = pService->createCharacteristic(
      ODOMETER_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE
  );
  odometerCharacteristic->setCallbacks(new MyCallbacks());
  odometerCharacteristic->setValue((uint8_t*)&odometer_corrections, 1);

  otaCharacteristic = pService->createCharacteristic(
      OTA_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );
  otaCharacteristic->setCallbacks(new OTACharacteristicCallbacks());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();
  oAdvertisementData.setManufacturerData(std::string((char*)manufacturerData, sizeof(manufacturerData)));
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  
  // Print the advertised UUID
  Serial.println("Advertising with UUID:");
  Serial.println(SERVICE_UUID);
  
  Serial.println("Waiting for a client connection to notify...");
}

void loop() {
  if (new_pulse) {
    new_pulse = false;
    bike_stopped = false;
  } else if (is_bike_stopped()) {
    bike_stopped = true;
    delay(10000);
    if (odometer_corrections) {
      input_offset_pulses();
    }
  }
  Serial.printf("Actual Pulse Count: %u, Spoofed Pulse Count: %u\n", actual_pulse_count, spoofed_pulse_count);
  delay(1000);
}

void core1_entry() {
  while (true) {
    if (!bike_stopped) {
      digitalWrite(PULSE_PIN, HIGH);
      delay(PULSE_WIDTH_MS);
      digitalWrite(PULSE_PIN, LOW);
      delay(current_interval > MIN_INTERVAL_MS ? current_interval - PULSE_WIDTH_MS : MIN_INTERVAL_MS - PULSE_WIDTH_MS);
      spoofed_pulse_count++;
    }
  }
}

bool debounce_check(uint32_t last_interrupt_time) {
  uint32_t current_time = millis();
  return (current_time - last_interrupt_time) >= DEBOUNCE_TIME_MS;
}

void gpio_callback() {
  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = millis();

  if (!debounce_check(last_interrupt_time)) {
    return;
  }

  last_interrupt_time = current_time;

  if (digitalRead(SENSOR1_PIN) == HIGH) {
    if (last_pulse_time != 0) {
      uint32_t interval = current_time - last_pulse_time;
      current_interval = interval;
    }
    last_pulse_time = current_time;
    new_pulse = true;
    actual_pulse_count++;
  }
}

bool is_bike_stopped() {
  uint32_t current_time = millis();
  return (current_time - last_pulse_time) > 2000;
}

void input_offset_pulses() {
  while (actual_pulse_count > spoofed_pulse_count) {
    if (new_pulse) {
      return;
    }
    digitalWrite(PULSE_PIN, HIGH);
    delay(PULSE_WIDTH_MS);
    digitalWrite(PULSE_PIN, LOW);
    delay(MIN_INTERVAL_MS - PULSE_WIDTH_MS);
    spoofed_pulse_count++;
  }
}