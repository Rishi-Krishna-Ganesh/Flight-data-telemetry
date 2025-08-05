#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <SPI.h>
#include <RF24.h>

MPU6050 mpu;
Adafruit_BMP280 bmp;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(2, 3); // RX, TX

RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

struct SensorData {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  float pressure;
  float altitude;
  float latitude;
  float longitude;
};

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  Wire.begin();
  mpu.initialize();
  bmp.begin();

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_LOW);
  radio.stopListening();
}

void loop() {
  SensorData data;

  // MPU6050
  data.accX = mpu.getAccelerationX() / 16384.0;
  data.accY = mpu.getAccelerationY() / 16384.0;
  data.accZ = mpu.getAccelerationZ() / 16384.0;
  data.gyroX = mpu.getRotationX() / 131.0;
  data.gyroY = mpu.getRotationY() / 131.0;
  data.gyroZ = mpu.getRotationZ() / 131.0;

  // BMP280
  data.temperature = bmp.readTemperature();
  data.pressure = bmp.readPressure() / 100.0F;
  data.altitude = bmp.readAltitude(1013.25); // Sea level pressure

  // GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
  } else {
    data.latitude = 0.0;
    data.longitude = 0.0;
  }

  // Transmit
  radio.write(&data, sizeof(SensorData));

  delay(200);
}
