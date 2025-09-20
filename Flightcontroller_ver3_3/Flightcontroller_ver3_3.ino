#include <ps5Controller.h>
#include <Wire.h>
#include <ESP32Servo.h> // Standard Servo library for ESP32

volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw, AccXCalibration, AccYCalibration, AccZCalibration;

int ESCfreq = 500;
float PAngleRoll = 2;
float PAnglePitch = PAngleRoll;
float IAngleRoll = 0.5;
float IAnglePitch = IAngleRoll;
float DAngleRoll = 0.007;
float DAnglePitch = DAngleRoll;

float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.0088;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

uint32_t LoopTimer;
float t = 0.004; // Time cycle

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;

const int mot1_pin = 4;
const int mot2_pin = 14;
const int mot3_pin = 25;
const int mot4_pin = 13;

volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;
volatile float KalmanGainPitch, KalmanGainRoll;

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

// Kalman filters for angle mode
volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
volatile float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
volatile float Kalman1DOutput[] = {0, 0};
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

int16_t AccXLSB;
int16_t AccYLSB;
int16_t AccZLSB;
int16_t GyroX;
int16_t GyroY;
int16_t GyroZ;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement)
{
  KalmanState = KalmanState + (t * KalmanInput);
  KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4);                    // here 4 is the vairnece of IMU i.e 4 deg/s
  float KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3); // std deviation of error is 3 deg
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

void OnConnect()
{
  // Serial.println("PS5 Controller Connected!");
  digitalWrite(2, HIGH);
}

void OnDisconnect()
{
  digitalWrite(2, LOW);
}

void setup(void)
{
  Serial.begin(115200);
  ps5.begin("58:10:31:2f:c8:b5");
  pinMode(2, OUTPUT);
  ps5.attachOnConnect(OnConnect);
  ps5.attachOnDisconnect(OnDisconnect);

  // int led_time = 100;
  // pinMode(15, OUTPUT);
  // digitalWrite(15, LOW);
  // delay(led_time);
  // digitalWrite(15, HIGH);
  // delay(led_time);
  // digitalWrite(15, LOW);
  // delay(led_time);
  // digitalWrite(15, HIGH);
  // delay(led_time);
  // digitalWrite(15, LOW);
  // delay(led_time);
  // digitalWrite(15, HIGH);
  // delay(led_time);

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

  delay(1000);
  mot1.attach(mot1_pin, 1000, 2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);
  mot2.attach(mot2_pin, 1000, 2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);
  mot3.attach(mot3_pin, 1000, 2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  //  delay(500);
  // digitalWrite(15, LOW);
  // digitalWrite(15, HIGH);
  // delay(500);
  // digitalWrite(15, LOW);
  // delay(500);

  RateCalibrationRoll = -0.82;
  RateCalibrationPitch = 0.94;
  RateCalibrationYaw = -1.28;
  AccXCalibration = 0.04;
  AccYCalibration = -0.02;
  AccZCalibration = -0.02;

  LoopTimer = micros();
}

void loop(void)
{
  int RightStickX = (abs(ps5.RStickX()) < 13) ? 0 : ps5.RStickX();
  int RightStickY = (abs(ps5.RStickY()) < 13) ? 0 : ps5.RStickY();
  int LeftStickX = (abs(ps5.LStickX()) < 13) ? 0 : ps5.LStickX();
  // int LeftStickY = (abs(ps5.LStickY()) < 13) ? 0 : ps5.LStickY();

  if (ps5.isConnected())
  {
    InputThrottle = map(ps5.L2Value(), 0, 255, 1000, 2000); // ThrottleCutOff
    InputYaw = map(LeftStickX, -128, 127, -400, 400);
    InputPitch = map(RightStickY, -128, 127, -400, 400);
    InputRoll = map(RightStickX, -128, 127, -400, 400);
  }
  else
  {
    InputThrottle = InputYaw = InputPitch = InputRoll = 0;
  }

  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;
  complementaryAngleRoll = constrain(complementaryAngleRoll, -20, 20);
  complementaryAnglePitch = constrain(complementaryAnglePitch, -20, 20);

  MotorInput1 = constrain(InputThrottle + InputPitch + InputRoll - InputYaw, ThrottleIdle, 2000);
  MotorInput2 = constrain(InputThrottle - InputPitch + InputRoll + InputYaw, ThrottleIdle, 2000);
  MotorInput3 = constrain(InputThrottle - InputPitch - InputRoll - InputYaw, ThrottleIdle, 2000);
  MotorInput4 = constrain(InputThrottle + InputPitch - InputRoll + InputYaw, ThrottleIdle, 2000);

  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  uint32_t nextLoopTime = LoopTimer + (t * 1e6);
  while (micros() < nextLoopTime)
  {
  }
  LoopTimer = nextLoopTime;

  // while (micros() - LoopTimer < (t * 1000000));
  // LoopTimer = micros();

  // Serial.println(InputThrottle + InputPitch + InputRoll - InputYaw);//MotorInput1
  // Serial.println(InputThrottle - InputPitch + InputRoll + InputYaw);//MotorInput2);
  // Serial.println(InputThrottle - InputPitch - InputRoll - InputYaw);//MotorInput3);
  // Serial.println(InputThrottle + InputPitch - InputRoll + InputYaw);//MotorInput4);
  // Serial.println("--------------------------------");
  // Serial.println(InputPitch);
  // Serial.println(InputRoll);
  // Serial.println(InputYaw);
  // Serial.println(InputThrottle);
  // Serial.println("--------------------------------");

  // Serial.println(ps5.RStickX());
  // Serial.println(ps5.RStickY());
  // Serial.println(ps5.LStickY());
  // Serial.println(ps5.LStickX());
  // Serial.println("--------------------------------");

  // Serial.println(GyroX);
  // Serial.println(GyroY);
  // Serial.println(GyroZ);
  // Serial.println(AccXLSB);
  // Serial.println(AccYLSB);
  // Serial.println(AccZLSB);
  // Serial.println("--------------------------------");
  // delay(250);
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

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

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  // // Inlined Kalman Filter computation in the loop
  // KalmanAngleRoll += t * RateRoll;
  // KalmanUncertaintyAngleRoll += t * t * 16; // Variance of IMU (4 deg/s) squared
  // KalmanGainRoll = KalmanUncertaintyAngleRoll / (KalmanUncertaintyAngleRoll + 9); // Error variance (3 deg) squared
  // KalmanAngleRoll += KalmanGainRoll * (AngleRoll - KalmanAngleRoll);
  // KalmanUncertaintyAngleRoll *= (1 - KalmanGainRoll);

  // // Set output for Roll Kalman
  // Kalman1DOutput[0] = KalmanAngleRoll;
  // Kalman1DOutput[1] = KalmanUncertaintyAngleRoll;

  // // Inlined Kalman Filter computation for Pitch
  // KalmanAnglePitch += t * RatePitch;
  // KalmanUncertaintyAnglePitch += t * t * 16; // Variance of IMU (4 deg/s) squared
  // KalmanGainPitch = KalmanUncertaintyAnglePitch / (KalmanUncertaintyAnglePitch + 9); // Error variance (3 deg) squared
  // KalmanAnglePitch += KalmanGainPitch * (AnglePitch - KalmanAnglePitch);
  // KalmanUncertaintyAnglePitch *= (1 - KalmanGainPitch);

  // // Set output for Pitch Kalman
  // Kalman1DOutput[0] = KalmanAnglePitch;
  // Kalman1DOutput[1] = KalmanUncertaintyAnglePitch;

  // KalmanAngleRoll = (KalmanAngleRoll > 20) ? 20 : ((KalmanAngleRoll < -20) ? -20 : KalmanAngleRoll);
  // KalmanAnglePitch = (KalmanAnglePitch > 20) ? 20 : ((KalmanAnglePitch < -20) ? -20 : KalmanAnglePitch);
}

void PID_calculation(void)
{

  // Inlined PID equation for Roll
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // Compute errors
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Axis PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
  PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

  // Update output and previous values for Roll
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Axis PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
  PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

  // Update output and previous values for Pitch
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Axis PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw); // Clamp ItermYaw to [-400, 400]
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
  PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw); // Clamp PIDOutputYaw to [-400, 400]

  // Update output and previous values for Yaw
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;
}
