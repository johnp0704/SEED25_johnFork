#include <Wire.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X distanceSensor;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  if (distanceSensor.begin() != 0)
  {
    Serial.println("VL53L1X not detected");
    while (1);
  }

  // ---- Narrowest FOV configuration ----

  // Short mode = better precision, narrower effective FOV
  distanceSensor.setDistanceModeShort();

  // Timing budget (ms)
  distanceSensor.setTimingBudgetInMs(50);

  // Intermeasurement period (ms)
  distanceSensor.setIntermeasurementPeriod(55);

  // ROI:
  // 4x4 SPADs, centered
  // Optical center = 199 (center of 16x16 SPAD array)
  distanceSensor.setROI(4, 4, 199);

  distanceSensor.startRanging();

  Serial.println("VL53L1X ready (narrow FOV)");
}

void loop()
{
  if (distanceSensor.checkForDataReady())
  {
    uint16_t distance = distanceSensor.getDistance();
    uint8_t status = distanceSensor.getRangeStatus();

    // Serial.print("Distance (mm): ");
    Serial.println(distance);
    // Serial.print("  Status: ");
    // Serial.println(status);

    distanceSensor.clearInterrupt();
  }

  delay(10);
}
