#include <ps5Controller.h>
#include <Wire.h>
#include <ESP32Servo.h>  // Standard Servo library for ESP32

int ESCfreq = 450;
int RightStickX, LeftStickX, RightStickY, LeftStickY, L2_Value, R2_Value;
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;
volatile float accelAngleR, accelAngleP, accelAngleY, gyroRateY, gyroRateX, gyroRateZ, filteredRollAngle, filteredPitchAngle, filteredYawAngle;

const unsigned long SIGNAL_TIMEOUT = 500000; // 500ms (500,000µs)
// unsigned long lastSignalTime;  // Tracks last received command time

float PAngleRoll = 2; //2
float IAngleRoll = 0.5; //0.5
float DAngleRoll = 0.007; //0.007
float PAnglePitch = PAngleRoll;
float IAnglePitch = IAngleRoll;
float DAnglePitch = DAngleRoll;

float PRateRoll = 0.6; //0.625
float IRateRoll = 0.7; //2.1
float DRateRoll = 0.0005; //0.0088

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t = 0.004;  // Time cycle

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 25;
const int mot2_pin = 14;
const int mot3_pin = 4;
const int mot4_pin = 13;

bool motorsArmed = false;
bool reset = false;
bool full = false;
uint32_t current_time;

volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

float angle = 0.0, bias = 0.0;
float P[2][2] = {{1, 0}, {0, 1}};
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float dt = 0.01;  // Time step (10ms)


int ThrottleIdle = 1002;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = { 0, 0, 0 };

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = { 0, 0 };
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4 ;

bool failsafe = false;
// void triggerFailsafe(){
//   if (InputThrottle > ThrottleIdle){
//   InputThrottle = InputThrottle-100;
//   // delay(100);
//   }
// }

float kalmanFilter(float newAngle, float newRate) {
    // Prediction Step
    float rate = newRate - bias;
    angle += dt * rate;
    
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update Step
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    float y = newAngle - angle;  
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}
void gyro_signal(){
  
  //enter your loop code here
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;


  // RateRoll = RateRoll < 2 ? 0 : RateRoll;
  // RatePitch = RatePitch < 2 ? 0 : RatePitch;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  accelAngleR = atan2(AccY, AccZ) * 180 / PI;  // Estimate roll angle from accelerometer
  gyroRateX = GyroX / 131.0;  // Convert gyro rate to deg/sec
  filteredRollAngle = kalmanFilter(accelAngleR, gyroRateX);

  accelAngleP = atan2(AccX, AccZ) * 180 / PI;  // Estimate pitch angle from accelerometer
  gyroRateY = GyroY / 131.0;  // Convert gyro rate to deg/sec
  filteredPitchAngle = kalmanFilter(accelAngleP, gyroRateY);    

  accelAngleY = atan2(AccX, AccY) * 180 / PI;  // Estimate pitch angle from accelerometer
  gyroRateZ = GyroZ / 131.0;  // Convert gyro rate to deg/sec
  filteredYawAngle = kalmanFilter(accelAngleY, gyroRateZ);
    
  
  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;
  // Clamping complementary filter roll angle to ±20 degrees
  complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
  complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);
  
}

void setup(void) {
  Serial.begin(115200);
  ps5.begin("58:10:31:2f:c8:b5");
  // lastSignalTime = micros();
  pinMode(2, OUTPUT);
  Wire.setClock(400000);
  Wire.begin();
  delay(150);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(100);
  mot1.attach(mot1_pin, 1000, 2000);
  delay(500);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin, 1000, 2000);
  delay(500);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin, 1000, 2000);
  delay(500);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(500);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);

  // new gy87
  RateCalibrationRoll = -2.80;//-2.80
  RateCalibrationPitch = 0.42;//0.42
  RateCalibrationYaw = -0.18;//0.18
  AccXCalibration = 0.03;
  AccYCalibration = -0.03;
  AccZCalibration = 0.03;

// RateCalibrationRoll=-2.90;
// RateCalibrationPitch=0.45;
// RateCalibrationYaw=-0.17;
// AccXCalibration=0.03;
// AccYCalibration=-0.03;
// AccZCalibration=0.03;
  //old gy87
  // RateCalibrationRoll = -1.05;
  // RateCalibrationPitch = 1.15;
  // RateCalibrationYaw = -1.28;
  // AccXCalibration = 0.03;
  // AccYCalibration = -0.01;
  // AccZCalibration = -0.02;

  LoopTimer = micros();
}

void loop(void) {
  if (ps5.isConnected()) {
    digitalWrite(2, HIGH);  // Turn on LED when connected
    failsafe = false;
    RightStickX = (abs(ps5.RStickX()) < 15) ? 0 : ps5.RStickX();
    RightStickY = (abs(ps5.RStickY()) < 15) ? 0 : ps5.RStickY();
    // LeftStickX = (abs(ps5.LStickX()) < 0) ? 0 : ps5.LStickX();
    LeftStickX = ps5.LStickX();
    LeftStickY = ps5.LStickY();
    // LeftStickY = (abs(ps5.LStickY()) < 10) ? 0 : ps5.LStickY();
    // lastSignalTime = micros(); // Reset the failsafe timer

  } else {  
    digitalWrite(2, LOW);  // Turn off LED when disconnected
    RightStickX = 0;
    RightStickY = 0;
    LeftStickX = 0;
    // failsafe = true;
    LeftStickY = -127;
  }  

  int L2_Value = ps5.L2Value();       // Read L2 trigger (0-255)
  int R2_Value = ps5.R2Value();       // Read R2 trigger (0-255)
  bool circlePressed = ps5.Circle();  // Read Circle button state
  bool crossPressed = ps5.Cross();    // Read Cross button state
  bool resetPressed = ps5.Square();   // Read square button state
  bool trianglePressed = ps5.Triangle();   // Read square button state

  // Arm motors if Circle is pressed
  if (failsafe) {
    if (InputThrottle > ThrottleIdle){
      for(int i = LeftStickY; i > -127 ; i++ ){
        LeftStickY -= 100; 
        delay(100);
        }
    }
  }

  if (circlePressed) {
    motorsArmed = true;
    // Serial.println("Motors Armed!");
    }

  // Disarm motors if Cross is pressed
  if (crossPressed) {
    motorsArmed = false;
    // Serial.println("Motors Disarmed!");
    }

  // if (trianglePressed) {
  //   // motorsArmed = true;
  //   full = true;
  //   delay(10); 
  // } else {
  //   // motorsArmed = false;
  //   full = false;
  // }

  if (resetPressed) {
    reset = true;
    delay(10); 
  } else {
    reset = false;
  }

  gyro_signal();

  // ps5 button values
  DesiredAngleRoll = map(RightStickX, -128, 127, -20, 20);   // 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = map(RightStickY, -128, 127, -20, 20);  //  0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = map(LeftStickY, -128, 127, 1000, 2000);      // ReceiverValue[2];
  // DesiredRateYaw = map(LeftStickX, -128, 127, -75, 75);        // 0.15 * (ReceiverValue[3] - 1500);

  if ((R2_Value > 0 && L2_Value > 0 )|| (R2_Value == 0 && L2_Value == 0)) {
    DesiredRateYaw = 0;  // Both triggers pressed, neutral yaw
  }
  else {
    if (R2_Value > 0) {
      DesiredRateYaw = map(R2_Value, 0, 255, 0, 25);  // R2 pressed, right yaw (0 to 75)
    }
    if (L2_Value > 0) {
      DesiredRateYaw = (map(L2_Value, 0, 255, 0, -25));  // L2 pressed, left yaw (-75 to 0)
    }
  }

  // Inlined PID equation for Roll 
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = (ItermRoll > 450) ? 450 : ((ItermRoll < -450) ? -450 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 450) ? 450 : ((PIDOutputRoll < -450) ? -450 : PIDOutputRoll);
  DesiredRateRoll = (PIDOutputRoll);
  PrevErrorAngleRoll = -ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = (ItermPitch > 450) ? 450 : ((ItermPitch < -450) ? -450 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 450) ? 450 : ((PIDOutputPitch < -450) ? -450 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = -ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Compute errors
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  // ErrorRateRoll = DesiredRateRoll - filteredRollAngle;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Axis PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 450) ? 450 : ((ItermRoll < -450) ? -450 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 450) ? 450 : ((PIDOutputRoll < -450) ? -450 : PIDOutputRoll);

  // Update output and previous values for Roll
  InputRoll = (PIDOutputRoll);
  PrevErrorRateRoll = -ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 450) ? 450 : ((ItermPitch < -450) ? -450 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 450) ? 450 : ((PIDOutputPitch < -450) ? -450 : PIDOutputPitch);

  // Update output and previous values for Pitch
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = -ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 450) ? 450 : ((ItermYaw < -450) ? -450 : ItermYaw);  // Clamp ItermYaw to [-450, 450]
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 450) ? 450 : ((PIDOutputYaw < -450) ? -450 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-450, 450]

  // Update output and previous values for Yaw
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = -ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  if (InputThrottle > 1800) {    InputThrottle = 1800;  }

  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw);  // front right - counter clockwise
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw);  // rear right - clockwise
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw);  // rear left  - counter clockwise
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw);  //front left - clockwise

  if (MotorInput1 > 1900) {    MotorInput1 = 1900;  }
  if (MotorInput2 > 1900) {    MotorInput2 = 1900;  }
  if (MotorInput3 > 1900) {    MotorInput3 = 1900;  }
  if (MotorInput4 > 1900) {    MotorInput4 = 1900;  }
  if (MotorInput1 < ThrottleIdle) {    MotorInput1 = ThrottleIdle;  }
  if (MotorInput2 < ThrottleIdle) {    MotorInput2 = ThrottleIdle;  }
  if (MotorInput3 < ThrottleIdle) {    MotorInput3 = ThrottleIdle;  }
  if (MotorInput4 < ThrottleIdle) {    MotorInput4 = ThrottleIdle;  }

  // If motors are disarmed, cut throttle and reset PID values
  // if (full)
  // {

  // // MotorInput1 = 2000;
  // // MotorInput2 = 2000;
  // // MotorInput3 = 2000;
  // // MotorInput4 = 2000;
  // }

  if (!motorsArmed)
  {   
  MotorInput1 = ThrottleCutOff;
  MotorInput2 = ThrottleCutOff;
  MotorInput3 = ThrottleCutOff;
  MotorInput4 = ThrottleCutOff;

  PrevErrorRateRoll=0; PrevErrorRatePitch=0; PrevErrorRateYaw=0;
  PrevItermRateRoll=0; PrevItermRatePitch=0; PrevItermRateYaw=0;
  PrevErrorAngleRoll=0; PrevErrorAnglePitch=0;    
  PrevItermAngleRoll=0; PrevItermAnglePitch=0;
  }
  if (!reset) {
    PrevErrorRateRoll = 0;
    PrevErrorRatePitch = 0;
    PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0;
    PrevItermRatePitch = 0;
    PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0;
    PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0;
    PrevItermAnglePitch = 0;
  }
  // Calculate motor control values directly
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

// // // serial print values 
//     Serial.print(ErrorAngleRoll); Serial.print(" , ");
//     Serial.print(PtermRoll); Serial.print(" , ");
//     Serial.print(ItermRoll); Serial.print(" , ");
//     Serial.print(DtermRoll); Serial.print(" , ");
//     Serial.print(PIDOutputRoll); Serial.print(" , ");
    
//     Serial.print(ErrorAnglePitch); Serial.print(" , ");
//     Serial.print(PtermPitch); Serial.print(" , ");
//     Serial.print(ItermPitch); Serial.print(" , ");
//     Serial.print(DtermPitch); Serial.print(" , ");
//     Serial.print(PIDOutputPitch); Serial.print(" , ");

//     Serial.print(ErrorRateYaw); Serial.print(" , ");
//     Serial.print(PtermYaw); Serial.print(" , ");
//     Serial.print(ItermYaw); Serial.print(" , ");
//     Serial.print(DtermYaw); Serial.print(" , ");
//     Serial.print(PIDOutputYaw);

    Serial.println(); // Move to next line for Serial Plotter
    Serial.print(MotorInput1); Serial.print(" , ");
    Serial.print(MotorInput2); Serial.print(" , ");
    Serial.print(MotorInput3); Serial.print(" , ");
    Serial.print(MotorInput4); Serial.print(" , ");
  // //Motor PWMs in us
  // Serial.print("MotVals- ");
  // Serial.print(MotorInput1);
  // Serial.print("  ");
  // Serial.print(MotorInput2);
  // Serial.print("  ");
  // Serial.print(MotorInput3);
  // Serial.print("  ");
  // Serial.print(MotorInput4);
  // Serial.print(" -- ");

  //   //Motor PWMs in us
  // Serial.print("RemoteVals- ");
  // Serial.print(InputThrottle);
  // Serial.print(RightStickX);
  // Serial.print(" - ");
  // Serial.println(RightStickY);
  // // Serial.println(MotorInput1);
  // // Serial.print("  ");
  // Serial.print(DesiredAngleRoll);
  // Serial.print("  ");
  // Serial.println(DesiredAnglePitch);
  // Serial.print("  ");
  // Serial.print(DesiredRateYaw);
  // Serial.println("  ");
  // Serial.print(L2_Value);
  // Serial.print(ps5.L2Value());
  // Serial.print("  ");
  // Serial.print(R2_Value);
  // Serial.print(ps5.R2Value());
  // Serial.println(" -| ");

  // //IMU values
  // Serial.println("Acc values: ");
  // Serial.print("X:");
  // Serial.print(AccX);
  // Serial.print("  ");
  // Serial.print("Y:");
  // Serial.print(AccY);
  // Serial.print("  ");
  // Serial.print("Z:");
  // Serial.print(AccZ);
  // Serial.print(" -- ");
  // Serial.println("Gyro values: ");
  // Serial.print("X:");
  // Serial.print(GyroX);
  // Serial.print("  ");
  // Serial.print("Y:");
  // Serial.print(GyroY);
  // Serial.print("  ");
  // Serial.print("Z:");
  // Serial.print(GyroZ);
  // Serial.print(" -- ");
  // // Print the gyroscope values
  Serial.println(" Gyro values: ");
  Serial.print(RateRoll);
  Serial.print("   ");
  Serial.print(RatePitch);
  Serial.print("   ");
  Serial.print(RateYaw);
  Serial.print("   ");

  Serial.println(" -- ");

  // // PID outputs
  // Serial.print(" PID O/P ");
  // Serial.print(InputPitch);
  // Serial.print("  ");
  // Serial.print(InputRoll);
  // Serial.print("  ");
  // Serial.print(InputYaw);
  // Serial.print(" -- ");

  // // Angles from MPU
  // Serial.print("ARoll:");
  // Serial.print(AngleRoll);
  // Serial.print("  ");
  // Serial.print("APitch:");
  // Serial.print(AnglePitch);

  // Serial.print("CompARoll: ");
  // Serial.print(complementaryAngleRoll);
  // Serial.print("CompAPitch: ");
  // Serial.print(complementaryAnglePitch);
  // Serial.print(" ");

  // Serial.print(" RateRoll: ");
  // Serial.print(RateRoll);
  // Serial.print("|- filteredRollAngle: ");
  // Serial.print(filteredRollAngle);
  // Serial.print("  ");

  // // serial plotter comparison
  // Serial.print(complementaryAngleRoll);
  // Serial.print(" ");
  // Serial.println(complementaryAnglePitch);
  delay(100);
  reset = false;
  // if (micros() - lastSignalTime > SIGNAL_TIMEOUT) {
  //       // triggerFailsafe(); // Take action if no signal for 500ms
  //       if (InputThrottle > ThrottleIdle){
  //       InputThrottle -= InputThrottle-100;
  //   }
  //   }
  while (micros() - LoopTimer < (t * 1000000))
    ;
  {
    LoopTimer = micros();
  }
  }