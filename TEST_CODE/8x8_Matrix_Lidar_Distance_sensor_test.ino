#include <Wire.h>
#include "DFRobot_MatrixLidar.h"

#define SDA_PIN 8
#define SCL_PIN 9

// I2C address 0x30 (A0=0, A1=0)
DFRobot_MatrixLidar_I2C lidar( 0x30,&Wire);

uint16_t distanceData[64];

void setup()
{
  Serial.begin(115200);
  delay(2000);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);  // 400kHz I2C (recommended)

  Serial.println("Initializing Matrix LiDAR...");

  while (lidar.begin() != 0) {
    Serial.println("Sensor not detected. Check wiring.");
    delay(1000);
  }

 lidar.setRangingMode(eMatrix_8X8);  // Set full 8x8 resolution

  Serial.println("Sensor ready.");
}

void loop()
{
  if (lidar.getAllData(distanceData) == 0) {

    Serial.println("----- 8x8 Distance (mm) -----");

    for (int row = 0; row < 8; row++) {
      for (int col = 0; col < 8; col++) {
        Serial.print(distanceData[row * 8 + col]);
        Serial.print("\t");
      }
      Serial.println();
    }

    Serial.println("-----------------------------\n");
  }

  delay(100);
}