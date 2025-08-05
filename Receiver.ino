#include <SPI.h>
#include <RF24.h>

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
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    SensorData data;
    radio.read(&data, sizeof(SensorData));

    Serial.println("----Sensor Data----");
    Serial.print("Accel: ");
    Serial.print(data.accX); Serial.print(", ");
    Serial.print(data.accY); Serial.print(", ");
    Serial.println(data.accZ);

    Serial.print("Gyro: ");
    Serial.print(data.gyroX); Serial.print(", ");
    Serial.print(data.gyroY); Serial.print(", ");
    Serial.println(data.gyroZ);

    Serial.print("Temp: "); Serial.println(data.temperature);
    Serial.print("Pressure: "); Serial.println(data.pressure);
    Serial.print("Altitude: "); Serial.println(data.altitude);

    Serial.print("Lat: "); Serial.print(data.latitude);
    Serial.print(" | Lon: "); Serial.println(data.longitude);
    Serial.println("-------------------\n");
  }
}
