#include <Wire.h>
#include <SparkFun_VL53L1X.h>

SFEVL53L1X distanceSensor;

// ---------------- FIR FILTER ----------------
constexpr int FIR_TAPS = 19;

// FIR coefficients (exactly as provided)
const float firCoeff[FIR_TAPS] = {
    0.000000000000000000,
    0.000350641163010758,
    -0.000851543223402231,
    -0.000864389927910587,
    0.009654295384169582,
    -0.031080453921735359,
    0.066713360633447072,
    -0.109776375699606535,
    0.145853281871451579,
    0.840002367441151310,
    0.145853281871451579,
    -0.109776375699606576,
    0.066713360633447100,
    -0.031080453921735359,
    0.009654295384169589,
    -0.000864389927910589,
    -0.000851543223402232,
    0.000350641163010758,
    0.000000000000000000,
};

float firBuffer[FIR_TAPS] = {0};
int firIndex = 0;

// ---------------- SETUP ----------------
void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);

  if (distanceSensor.begin() != 0)
  {
    while (1);
  }

  distanceSensor.setDistanceModeShort();
  distanceSensor.setTimingBudgetInMs(20);        // 50 Hz
  distanceSensor.setIntermeasurementPeriod(20);
  distanceSensor.setROI(4, 4, 199);

  distanceSensor.startRanging();
}

// ---------------- FIR FUNCTION ----------------
float applyFIR(float input)
{
  firBuffer[firIndex] = input;

  float acc = 0.0f;
  int idx = firIndex;

  for (int i = 0; i < FIR_TAPS; i++)
  {
    acc += firCoeff[i] * firBuffer[idx];
    idx = (idx == 0) ? FIR_TAPS - 1 : idx - 1;
  }

  firIndex++;
  if (firIndex >= FIR_TAPS) firIndex = 0;

  return acc;
}

// ---------------- LOOP ----------------
void loop()
{
  static uint32_t lastSample = 0;

  if (millis() - lastSample >= 20) // 50 Hz
  {
    lastSample = millis();

    if (distanceSensor.checkForDataReady())
    {
      float raw = (float)distanceSensor.getDistance();
      float filtered = applyFIR(raw);

      Serial.println(filtered);

      distanceSensor.clearInterrupt();
    }
  }
}
