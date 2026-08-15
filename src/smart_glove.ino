#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;

const int flexPin1 = 35; // index finger
const int flexPin2 = 32; // middle finger
const int flexPin3 = 34; // ring finger

const int flex1Straight = 1050;
const int flex1Bent = 820;
const int flex2Straight = 955;
const int flex2Bent = 735;
const int flex3Straight = 320;
const int flex3Bent = 587;

float smoothFlex1 = 0, smoothFlex2 = 0, smoothFlex3 = 0;
const float flexAlpha = 0.1;

// FSR pins — p8=index(D33), p6=middle(D25), p2=palm(D27)
const int fsrIndex = 33;
const int fsrMiddle = 25;
const int fsrPalm = 27;

const int WINDOW_SIZE = 25;
const float TREMOR_THRESHOLD = 0.3;
const int TREMOR_CROSSINGS = 8;

float gyroX_history[WINDOW_SIZE];
float gyroY_history[WINDOW_SIZE];
float gyroZ_history[WINDOW_SIZE];
int historyIndex = 0;
bool bufferFull = false;

float baselineGrip = 0;
bool baselineSet = false;
int baselineSampleCount = 0;
float baselineSum = 0;
float fatigueScore = 0;
const int BASELINE_SAMPLES = 50;

void calibrateSensor() {
  Serial.println("Calibrating... keep the sensor still!");
  int numSamples = 200;
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0, sumAZ = 0;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumGX += g.gyro.x;
    sumGY += g.gyro.y;
    sumGZ += g.gyro.z;
    sumAX += a.acceleration.x;
    sumAY += a.acceleration.y;
    sumAZ += a.acceleration.z;
    delay(5);
  }

  gyroX_offset = sumGX / numSamples;
  gyroY_offset = sumGY / numSamples;
  gyroZ_offset = sumGZ / numSamples;
  accelX_offset = sumAX / numSamples;
  accelY_offset = sumAY / numSamples;
  accelZ_offset = (sumAZ / numSamples) - 9.8;

  Serial.println("Calibration done!");
}

float toPercent(float raw, int straightVal, int bentVal) {
  float percent;
  if (bentVal > straightVal) {
    percent = (raw - straightVal) / (float)(bentVal - straightVal) * 100.0;
  } else {
    percent = (straightVal - raw) / (float)(straightVal - bentVal) * 100.0;
  }
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  return percent;
}

int countZeroCrossings(float* history, int size) {
  int crossings = 0;
  for (int i = 1; i < size; i++) {
    if (history[i-1] >= TREMOR_THRESHOLD && history[i] < -TREMOR_THRESHOLD) crossings++;
    if (history[i-1] <= -TREMOR_THRESHOLD && history[i] > TREMOR_THRESHOLD) crossings++;
  }
  return crossings;
}

bool detectTremor() {
  if (!bufferFull) return false;
  int crossX = countZeroCrossings(gyroX_history, WINDOW_SIZE);
  int crossY = countZeroCrossings(gyroY_history, WINDOW_SIZE);
  int crossZ = countZeroCrossings(gyroZ_history, WINDOW_SIZE);
  return (crossX >= TREMOR_CROSSINGS || crossY >= TREMOR_CROSSINGS || crossZ >= TREMOR_CROSSINGS);
}

float getAverageGrip(int index, int middle, int palm) {
  return (index + middle + palm) / 3.0;
}

float computeFatigue(float currentGrip, bool tremor) {
  if (!baselineSet) return 0;
  if (baselineGrip < 10) return 0;

  float gripDrop = (baselineGrip - currentGrip) / baselineGrip * 100.0;
  if (gripDrop < 0) gripDrop = 0;
  if (gripDrop > 100) gripDrop = 100;

  float tremorContribution = tremor ? 25.0 : 0.0;
  float score = (gripDrop * 0.75) + tremorContribution;
  if (score > 100) score = 100;
  return score;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found - check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 found!");

  calibrateSensor();

  smoothFlex1 = analogRead(flexPin1);
  smoothFlex2 = analogRead(flexPin2);
  smoothFlex3 = analogRead(flexPin3);

  for (int i = 0; i < WINDOW_SIZE; i++) {
    gyroX_history[i] = 0;
    gyroY_history[i] = 0;
    gyroZ_history[i] = 0;
  }

  Serial.println("Squeeze all FSRs hard for 10 seconds to set baseline!");
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x - accelX_offset;
  float ay = a.acceleration.y - accelY_offset;
  float az = a.acceleration.z - accelZ_offset;

  float gx = g.gyro.x - gyroX_offset;
  float gy = g.gyro.y - gyroY_offset;
  float gz = g.gyro.z - gyroZ_offset;

  gyroX_history[historyIndex] = gx;
  gyroY_history[historyIndex] = gy;
  gyroZ_history[historyIndex] = gz;
  historyIndex = (historyIndex + 1) % WINDOW_SIZE;
  if (historyIndex == 0) bufferFull = true;

  bool tremor = detectTremor();

  smoothFlex1 = smoothFlex1 + flexAlpha * (analogRead(flexPin1) - smoothFlex1);
  smoothFlex2 = smoothFlex2 + flexAlpha * (analogRead(flexPin2) - smoothFlex2);
  smoothFlex3 = smoothFlex3 + flexAlpha * (analogRead(flexPin3) - smoothFlex3);

  float pct1 = toPercent(smoothFlex1, flex1Straight, flex1Bent);
  float pct2 = toPercent(smoothFlex2, flex2Straight, flex2Bent);
  float pct3 = toPercent(smoothFlex3, flex3Straight, flex3Bent);

  int index = analogRead(fsrIndex);
  int middle = analogRead(fsrMiddle);
  int palm = analogRead(fsrPalm);

  float avgGrip = getAverageGrip(index, middle, palm);

  if (!baselineSet) {
    baselineSum += avgGrip;
    baselineSampleCount++;
    if (baselineSampleCount >= BASELINE_SAMPLES) {
      baselineGrip = baselineSum / BASELINE_SAMPLES;
      baselineSet = true;
      Serial.print("Baseline grip set: ");
      Serial.println(baselineGrip);
    }
  }

  fatigueScore = computeFatigue(avgGrip, tremor);

  Serial.println("========== REHAB GLOVE ==========");

  Serial.println("--- MOVEMENT ---");
  Serial.print("Accel  X: "); Serial.print(ax, 2);
  Serial.print("  Y: "); Serial.print(ay, 2);
  Serial.print("  Z: "); Serial.println(az, 2);
  Serial.print("Gyro   X: "); Serial.print(gx, 2);
  Serial.print("  Y: "); Serial.print(gy, 2);
  Serial.print("  Z: "); Serial.println(gz, 2);

  Serial.println("--- FINGERS ---");
  Serial.print("Index:  "); Serial.print(pct1, 1); Serial.println("%");
  Serial.print("Middle: "); Serial.print(pct2, 1); Serial.println("%");
  Serial.print("Ring:   "); Serial.print(pct3, 1); Serial.println("%");

  Serial.println("--- GRIP PRESSURE ---");
  Serial.print("Index finger: "); Serial.println(index);
  Serial.print("Middle finger: "); Serial.println(middle);
  Serial.print("Palm:         "); Serial.println(palm);

  Serial.println("--- STATUS ---");
  Serial.print("Tremor:  "); Serial.println(tremor ? "YES ⚠️" : "None");
  Serial.print("Fatigue: "); Serial.print(fatigueScore, 1); Serial.println("%");
  if (!baselineSet) Serial.println(">> Squeeze hard to set baseline <<");

  Serial.println("=================================");
  Serial.println();

  delay(200);
}
