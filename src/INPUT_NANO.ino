#include <NewPing.h>
#include <Wire.h>
#include <MPU6050.h>

// Ultrasonic Sensor Pin Definitions
#define TRIG_PIN_FRONT  3
#define ECHO_PIN_FRONT  2
#define TRIG_PIN_RIGHT  4
#define ECHO_PIN_RIGHT  5
#define TRIG_PIN_LEFT   6
#define ECHO_PIN_LEFT   7
#define TRIG_PIN_BACK   8
#define ECHO_PIN_BACK   9

// Maximum distance to measure (in cm)
#define MAX_DISTANCE 2000000

// Create NewPing objects for each ultrasonic sensor
NewPing sonarFront(TRIG_PIN_FRONT, ECHO_PIN_FRONT, MAX_DISTANCE);
NewPing sonarRight(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT, MAX_DISTANCE);
NewPing sonarLeft(TRIG_PIN_LEFT, ECHO_PIN_LEFT, MAX_DISTANCE);
NewPing sonarBack(TRIG_PIN_BACK, ECHO_PIN_BACK, MAX_DISTANCE);

// MPU6050 sensor object
MPU6050 mpu;

// Variables to store offsets for zeroing the MPU6050 values
int16_t ax_offset, ay_offset, az_offset;
int16_t gx_offset, gy_offset, gz_offset;

void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Check if the MPU6050 is connected correctly
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1); // Stay here forever if the MPU6050 is not connected
  }

  // Calibrate MPU6050 to zero initial values
  calibrateMPU6050();
}

void loop() {
  // Get distances from ultrasonic sensors
  unsigned int distanceFront = sonarFront.ping_cm();
  unsigned int distanceRight = sonarRight.ping_cm();
  unsigned int distanceLeft = sonarLeft.ping_cm();
  unsigned int distanceBack = sonarBack.ping_cm();

  // Get acceleration and gyroscope data from MPU6050
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Apply offsets to zero the values
  ax -= ax_offset;
  ay -= ay_offset;
  az -= az_offset;
  gx -= gx_offset;
  gy -= gy_offset;
  gz -= gz_offset;

  // Send the sensor data to the Serial port in a formatted string
  Serial.print("F:"); Serial.print(distanceFront); // Front distance
  Serial.print(",R:"); Serial.print(distanceRight); // Right distance
  Serial.print(",L:"); Serial.print(distanceLeft); // Left distance
  Serial.print(",B:"); Serial.print(distanceBack); // Back distance
  Serial.print(",AX:"); Serial.print(ax); // Accelerometer X
  Serial.print(",AY:"); Serial.print(ay); // Accelerometer Y
  Serial.print(",AZ:"); Serial.print(az); // Accelerometer Z
  Serial.print(",GX:"); Serial.print(gx); // Gyroscope X
  Serial.print(",GY:"); Serial.print(gy); // Gyroscope Y
  Serial.print(",GZ:"); Serial.println(gz); // Gyroscope Z

  delay(100); // Delay before next loop iteration
}

// Function to calibrate the MPU6050 by reading the initial offsets
void calibrateMPU6050() {
  int16_t ax, ay, az, gx, gy, gz;

  // Read the MPU6050 initial values
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Store the offsets
  ax_offset = ax;
  ay_offset = ay;
  az_offset = az;
  gx_offset = gx;
  gy_offset = gy;
  gz_offset = gz;

  Serial.println("MPU6050 calibrated. Offsets set to zero the initial values.");
}
