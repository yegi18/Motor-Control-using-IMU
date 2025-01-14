#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <I2Cdev.h>                          // Include I2C protocol
#include <MPU6050.h>                         // Include MPU-6050 sensor

/*****************************************************************************************************************************************************************/
#define DEGREE_METHOD_USING_MICROS

MPU6050 gyroAccelTemp;                       // Declare a MPU6050 object instance

//Define the properties of the MPU-6050 (IMU)
float gyroDegree;
/*If you have any Serial commands at baud rate < 98400, then the above value has to decrease*/
#define GYRO_Z_OFFSET 16                     // Determined using IMU_Zero under File > Examples > MPU6050

#ifdef DEGREE_METHOD_USING_MILLIS
  bool isFirstLoopComplete;                  // Declaring the isFirstLoopComplete boolean flag
  float previousTime;                        // Declaring the value to hold the time
#endif

#ifdef DEGREE_METHOD_USING_MICROS
  bool isFirstLoopComplete;                  // Declaring the isFirstLoopComplete boolean flag
  float previousTime;                        // Declaring the value to hold the time
#endif

#ifdef DEGREE_METHOD_USING_SAMPLE_RATE
  #define SAMPLE_RATE 1667                   // This is the default sampling rate in Hz of the MPU 6050
#endif
/*****************************************************************************************************************************************************************/

Adafruit_MPU6050 mpu;
/*****************************************************************************************************************************************************************/

Servo myServo;
/*****************************************************************************************************************************************************************/

// int pos1 = 0;
// void rotateServo(Servo motor, int targetPosition, int angularSpeed) {
//   mapSpeed = map(angularSpeed, 0, 30, 30, 0)
//   if (targetPosition  > pos) {
//     for (pos; pos <= targetPosition; pos += angularSpeed) {
//       motor.write(i);
//       pos1 = pos;
//       delay(mapSpeed);
//     }
//   }else{
//     for (int i = currentPosition; i > targetPosition; i += angularSpeed) {
//       motor.write(i);
//     }
//   }
// }


void setup(void) {
/*****************************************************************************************************************************************************************/
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
/*****************************************************************************************************************************************************************/

  Wire.begin();
  Serial.println("Initializing I2C devices...");
  gyroAccelTemp.initialize();

  // Verify IMU connection
  Serial.println("Testing device connections...");
  Serial.println(gyroAccelTemp.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  gyroAccelTemp.CalibrateGyro();                  // Fully calibrate gyro in about 15 loops
  Serial.println("Calibration complete");         // Notify when calibration is complete
  /* The following will NOT get rid of the noise produce by the IMU but they will ensure that
      the average values are at 0 to a certain extent
      (Refer to the IMU Error Determination code and the MPU6050 tutorial video for more details)
  */
  // Set Gyroscope Offsets
  gyroAccelTemp.setZGyroOffset(GYRO_Z_OFFSET);    // Set the Z gyroscope offset

  // IMPORTANT: If you do not calibrate after setting your offset, there will be error
  gyroAccelTemp.CalibrateGyro(6);                 // Fine Tuning purposes of gyroscope
  Serial.println("Fine Tuning Complete");         // Notify when fine tuning is complete

  #ifdef DEGREE_METHOD_USING_MILLIS
    isFirstLoopComplete = false;
  #endif

  #ifdef DEGREE_METHOD_USING_MICROS
    isFirstLoopComplete = false;
  #endif
/*****************************************************************************************************************************************************************/

int servoPin = 18;
myServo.attach(servoPin);
myServo.write(0);
}

void loop() {

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  // Serial.print("Acceleration X: ");
  // Serial.print(a.acceleration.x);
  // Serial.print(", Y: ");
  // Serial.print(a.acceleration.y);
  // Serial.print(", Z: ");
  // Serial.print(a.acceleration.z);
  // Serial.println(" m/s^2");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x*57.2958);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y*57.2958);
  // Serial.print(", Z: ");
  Serial.println("Rotation Z: ");
  Serial.print("Angular velocity: ");
  Serial.print(g.gyro.z*57.2958);
  Serial.println(" deg/s ");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");



/*****************************************************************************************************************************************************************/
    float gyroZ = gyroAccelTemp.getRotationZ() / 131.0;         // Get current Z axis orientation in degress per second

  #ifdef DEGREE_METHOD_USING_MILLIS
    /*
       The data extracted from the gyroscope is in degrees per second.
       The time taken to complete one loop is determined using millis()
       NOTE: The first value should not be considered as it is not possible
              to determine the time to complete one loop so a boolean flag is
              used
       NOTE_2: Millis() overflows after about 50 days

       1. Determine time taken to complete one loop
       2. Multiply that value by the gyroscope value extracted from the IMU
       3. Apply proper unit analysis
            --> millis() is in milliseconds (there are 1000 ms in 1 s)
            --> Data from gyroscope is in degree/s
     */
     if (isFirstLoopComplete) {
      float timeForOneLoop = millis() - previousTime;
      //Serial.println(timeForOneLoop);
      gyroDegree += gyroZ * timeForOneLoop / 1000.0;
     }
     // NOTE: Try and keep the following code close to the above if statement
     previousTime = millis();
     
     // Change the boolean flag to true to enable collection of gyroscope data
     if (!isFirstLoopComplete) {
      isFirstLoopComplete = true;
     }
  #endif

  #ifdef DEGREE_METHOD_USING_MICROS
  /*
     This is identical to the method using millis() but uses micros().
     It is more accurrate
     NOTE: micros() overflows after about 70 minutes

     1. Determine time taken to complete one loop
       2. Multiply that value by the gyroscope value extracted from the IMU
       3. Apply proper unit analysis
            --> micros() is in microseconds (there are 1000000 us in 1 s)
            --> Data from gyroscope is in degree/s
   */
  if (isFirstLoopComplete) {
      float timeForOneLoop = micros() - previousTime;
      //Serial.println(timeForOneLoop);
      gyroDegree += gyroZ * timeForOneLoop / 1000000.0;
     }
     // NOTE: Try and keep the following code close to the above if statement
     previousTime = micros();
     
     // Change the boolean flag to true to enable collection of gyroscope data
     if (!isFirstLoopComplete) {
      isFirstLoopComplete = true;
     }
  #endif
  
  #ifdef DEGREE_METHOD_USING_SAMPLE_RATE
    /*
       The data extracted from the gyroscope is in degrees per second.
       The Gyroscope sample rate is in units of Hz, which is second^-1.
          --> The sample rate is a predefined value
       1. Therefore, divide the data extracted by the gyroscope with the sample rate to get degrees
       2. Accumulate the gyroDegree calculated
    */
    gyroDegree += gyroZ / SAMPLE_RATE;
  #endif
  Serial.print("Angle: ");
  Serial.println(gyroDegree);
/*****************************************************************************************************************************************************************/

  Serial.println("");
  // rotateServo(myServo, gyroDegree, gyroDegree+90, gyroZ);
  myServo.write(gyroDegree+90);
  delay(50);
}