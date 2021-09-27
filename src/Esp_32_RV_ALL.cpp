#include <Arduino.h>

#include "Adafruit_VL53L0X.h"
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <Servo.h>
#include "Wire.h"

// Direction I2C
#define MPU 0x68 // IMU

// Conversions
#define A_R 16384.0            // 32768/2
#define G_R 131.0              // 32768/250
#define RAD_TO_DEG = 57.295779 // 180/PI

/*  Objects  */
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
TaskHandle_t Task1; // for other core
Servo servo1;
BluetoothSerial SerialBT;

/*  Pins  */
#define led 2

// MPU
// slc 22, sda 21

// Motor
#define rigth_A 34
#define rigth_B 35
#define rigth_E 13
#define rigth_Dir_1 14
#define rigth_Dir_2 12
#define left_A 4
#define left_B 5
#define left_E 25
#define left_Dir_1 26
#define left_Dir_2 27

// Servo
#define servoPin 18

// US
#define trigPin 5
#define echoPin 4

/* Variables */
// Servo
int distance;
long duration;

// US
int posDegrees;
bool turnChange = true;

// Directions Motor
int forward[2] = {0, 1};
int back[2] = {1, 0};
int stop[2] = {0, 0};

// Temps
long lastLecture = 0;
long lastServo = 0;
long lastLaser = 0;
long lastMotorControl = 0;
long lastCarControl = 0;
int count = 0;

// Communication
String incoming = "";
String toSend = "";
bool in = false;

// Setting PWM properties
int PWM = 360;
const int freq = 100;
const int channelI = 8;
const int channelD = 9;
const int resolution = 10;

// MPU-6050
int16_t AcX, AcY, GyY, GyZ;

float Acc[2];
float Wp_MPU;
float Gy_Z;
float Gy_Y;
float Ac_X;
float Ac_Y;
float Angle_Z;
float Max_Angle_Y;
float Max_Vel_X;
float Vel_Y;
float Vp_MPU;
float Post_X;
float Post_Y;

String values;

// Encoder
const bool mov_forward[5][2] = {
    {0, 0},
    {1, 0},
    {1, 1},
    {0, 1},
    {0, 0},
};

const bool mov_back[5][2] = {
    {0, 0},
    {0, 1},
    {1, 1},
    {1, 0},
    {0, 0},
};

int now_microSegL;
int now_microSegR;

int enc_antLA;
int enc_antLB;
int enc_antRA;
int enc_antRB;

int enc_LA;
int enc_LB;
int enc_RA;
int enc_RB;

float W_encR = 0;
float W_encL = 0;
float Wd_encR = 0;
float Wd_encL = 0;
float Wr_ant = 0;
float Wl_ant = 0;
float Vd = 2.4;
float Wd = 0;
float Vp_Enc = 0;
float Wp_Enc = 0;
const float R = 2.4;
const float L = 12;
float Err_Wr_ant = 0;
float Err_Wl_ant = 0;
float tR;
float tL;
size_t contL = 1;
size_t contR = 1;
size_t cont = 1;

float Wr = 0;
float Wl = 0;
float x_MPU[3000];
float y_MPU[3000];
float theta_MPU[3000];
float x_Enc[3000];
float y_Enc[3000];
float theta_Enc[3000];

long prevTime;
long prevTimeEnc;
long prevTimeMPU;
float dt;

// Time between action (milliseconds)
int laserPeriod = 200;
int servoPeriod = 200;
int mpuPeriod = 10;
int carControlPeriod = 30;
int motorControlPeriod = 10;

/* Function declaration */
void codeForTask1(void *parameter);
void move(int Dir_R[2], int Dir_L[2], int PWM_R, int PWM_L);
void f_move_L(int Dir_L[2], int PWM_L);
void f_move_R(int Dir_R[2], int PWM_R);
void f_forward();
void f_back();
void f_right();
void f_left();
void f_stop();
void clear();
void encoder();
float processEnc(bool enc_antA, bool encA, bool enc_antB, bool encB, int now_microSeg);

void setup()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Serial.begin(1500000);
  Serial.println("VL53L0X test");
  if (!lox.begin())
  {
    Serial.println(F("Error al iniciar VL53L0X"));
  }

  xTaskCreatePinnedToCore(
      codeForTask1, // Task function.
      "Task_1",     // name of task.
      1000,         // Stack size of task
      NULL,         // parameter of the task
      1,            // priority of the task
      &Task1,       // Task handle to keep track of created task
      0);           // Core

  SerialBT.begin("ESP32"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  servo1.attach(servoPin);

  pinMode(led, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(rigth_A, INPUT);
  pinMode(rigth_B, INPUT);
  pinMode(rigth_Dir_1, OUTPUT);
  pinMode(rigth_Dir_2, OUTPUT);
  pinMode(left_A, INPUT);
  pinMode(left_B, INPUT);
  pinMode(left_Dir_1, OUTPUT);
  pinMode(left_Dir_2, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(channelI, freq, resolution);
  ledcSetup(channelD, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(left_E, channelI);
  ledcAttachPin(rigth_E, channelD);

  enc_antLA = digitalRead(left_A);
  enc_antLB = digitalRead(left_B);
  enc_antRA = digitalRead(rigth_A);
  enc_antRB = digitalRead(rigth_B);
}

void loop()
{
  long now = millis();

  /* if (now - lastLaser > laserPeriod)
  {
    lastLaser = now;
    VL53L0X_RangingMeasurementData_t measure;

    lox.rangingTest(&measure, false); // si se pasa true como parametro, muestra por puerto serie datos de debug
    if (measure.RangeStatus != 4)
    {
      distance = measure.RangeMilliMeter;
      // Serial.print("Distancia (mm): "+distance);
    }
    else
    {
      // Serial.println(" 1280");
      distance = 1280;
    }
  }
  */

  if (now - lastServo > servoPeriod)
  {
    lastServo = now;
    if (turnChange)
    {
      posDegrees = posDegrees + 45;
      if (posDegrees >= 180)
      {
        turnChange = false;
      }
    }
    else
    {
      posDegrees = posDegrees - 45;
      if (posDegrees <= 1)
      {
        turnChange = true;
      }
    }
    servo1.write(posDegrees);
  }

  now = millis();

  if (now - lastLecture >= mpuPeriod)
  {
    lastLecture = now;

    // Read accelerometer data
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();

    // Read gyroscope data
    Wire.beginTransmission(MPU);
    Wire.write(0x45);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 4, true);
    GyY = Wire.read() << 8 | Wire.read();
    GyZ = Wire.read() << 8 | Wire.read();

    Ac_X = AcX / A_R;
    Ac_Y = AcY / A_R;
    Gy_Z = GyZ / G_R;
    Gy_Y = GyY / G_R;

    dt = (millis() - prevTime) / 1000.0;
    prevTime = millis();

    // Integrals to obtain moviment in X
    float Vel_X = Ac_X * dt;

    if (Vel_X > Max_Vel_X)
    {
      Max_Vel_X = Vel_X;
    }

    if (Ac_Y * dt > Max_Vel_X)
    {
      // Integrals to obtain vel in Y
      Vp_MPU += Ac_Y * dt; // X carro
    }

    float Angle_Y = Gy_Y * dt;

    if (Angle_Y > Max_Angle_Y)
    {
      Max_Angle_Y = Angle_Y;
    }

    if (Gy_Z * dt > Max_Angle_Y)
    {
      Wp_MPU += Gy_Z * dt;
    }
  }

  if (now - lastCarControl >= carControlPeriod)
  {
    lastCarControl = now;

    Wr = Wr / (carControlPeriod / motorControlPeriod);
    Wl = Wl / (carControlPeriod / motorControlPeriod);

    Vp_Enc = Wr * (1.2) + Wl * (1.04347);
    Wp_Enc = Wr * (1.2) - Wl * (1.04347);

    float Vel_X_Enc = cos(theta_Enc[cont - 1] * 0.0174532925) * Vp_Enc;
    float Vel_Y_Enc = sin(theta_Enc[cont - 1] * 0.0174532925) * Vp_Enc;

    dt = (millis() - prevTimeEnc) / 1000.0;
    prevTimeEnc = millis();

    x_Enc[cont] = x_Enc[cont - 1] + dt * Vel_X_Enc * 6.41025641;
    y_Enc[cont] = y_Enc[cont - 1] + dt * Vel_Y_Enc * 6.41025641;
    theta_Enc[cont] = theta_Enc[cont - 1] + dt * Wp_Enc;
    /*
    Serial.print("Wr: ");
    Serial.print(Wr);
    Serial.print("  Vp: ");
    Serial.print(Vp_Enc);
    Serial.print("  VelX: ");
    Serial.print(Vel_X_Enc);
    Serial.print("  VelY: ");
    Serial.print(Vel_Y_Enc);
    Serial.print("  Wp: ");
    Serial.print(Wp_Enc);
    Serial.print("  x: ");
    Serial.print(x_Enc[cont]);
    Serial.print("  y: ");
    Serial.print(y_Enc[cont]);
    Serial.print("  tetaAct: ");
    Serial.println(theta_Enc[cont]);*/
    cont++;
    Wr = 0;
    Wl = 0;
  }

  if (now - lastMotorControl >= motorControlPeriod)
  {
    lastMotorControl = now;
    // Control rueda derecha
    Wd_encR = Vd / R + (Wd * L) / (2 * R);
    float Err_Wr = Wd_encR - W_encR;
    float salR = 2.1806 * Err_Wr - 1.9209 * Err_Wr_ant + Wr_ant;
    if (salR > 7)
    {
      salR = 7;
    }
    else if (salR < -7)
    {
      salR = -7;
    }
    if (Wd_encR > 0)
    {
      PWM = salR * 70 + 10;
      if (PWM < 0)
      {
        PWM = 0;
      }
      //f_move_L(forward, PWM);
    }
    else
    {
      PWM = salR * -70 + 10;
      if (PWM < 0)
      {
        PWM = 0;
      }
      //f_move_L(back, PWM);
    }

    //Serial.println(W_encR);
    //Serial.println(now);
    Wr_ant = salR;
    Err_Wr_ant = Err_Wr;

    // Control rueda izquierda
    Wd_encL = Vd / R - (Wd * L) / (2 * R);
    float Err_Wl = Wd_encL * 1.15 - W_encL;
    float salL = 2.2043 * Err_Wl - 1.9347 * Err_Wl_ant + Wl_ant;
    if (salL > 7)
    {
      salL = 7;
    }
    else if (salL < -7)
    {
      salL = -7;
    }

    if (Wd_encL > 0)
    {
      PWM = salL * 70 + 10;
      if (PWM < 0)
      {
        PWM = 0;
      }
      //f_move_R(forward, PWM);
    }
    else
    {
      PWM = salL * -70 + 10;
      if (PWM < 0)
      {
        PWM = 0;
      }
      //f_move_R(back, PWM);
    }
    //Serial.print(Wd_encL);
    //Serial.print("  ");
    //Serial.println(PWM);
    Wl_ant = salL;
    Err_Wl_ant = Err_Wl;
    Wr += W_encR;
    Wl += W_encL;

    Wp_MPU = Wp_MPU / (carControlPeriod / motorControlPeriod);
    Vp_MPU = Vp_MPU / (carControlPeriod / motorControlPeriod);

    float Vel_X_MPU = cos(theta_MPU[cont - 1] * 0.0174532925) * Vp_MPU;
    float Vel_Y_MPU = sin(theta_MPU[cont - 1] * 0.0174532925) * Vp_MPU;

    dt = (millis() - prevTimeMPU) / 1000.0;
    prevTimeMPU = millis();

    x_MPU[cont] = x_MPU[cont - 1] + dt * Vel_X_MPU * 6.41025641;
    y_MPU[cont] = y_MPU[cont - 1] + dt * Vel_Y_MPU * 6.41025641;
    theta_MPU[cont] = theta_MPU[cont - 1] + dt * Wp_MPU;

    Serial.print("Wr: ");
    Serial.print(Wr);
    Serial.print("  Vp: ");
    Serial.print(Vp_MPU);
    Serial.print("  VelX: ");
    Serial.print(Vel_X_MPU);
    Serial.print("  VelY: ");
    Serial.print(Vel_Y_MPU);
    Serial.print("  Wp: ");
    Serial.print(Wp_MPU);
    Serial.print("  x: ");
    Serial.print(x_MPU[cont]);
    Serial.print("  y: ");
    Serial.print(y_MPU[cont]);
    Serial.print("  tetaAct: ");
    Serial.println(theta_MPU[cont]);
    Vp_MPU = 0;
    Wp_MPU = 0;
  }

  if (in)
  {
    if (incoming == "B")
    {
      f_back();
    }
    if (incoming == "R")
    {
      f_right();
    }
    if (incoming == "L")
    {
      f_left();
    }
    if (incoming == "F")
    {
      digitalWrite(led, HIGH);
      f_forward();
    }
    if (incoming == "S")
    {
      digitalWrite(led, LOW);
      f_stop();
    }
    if (incoming == "a")
    {
      toSend = String(Angle_Z);
    }
    if (incoming == "d")
    {
      toSend = String(distance);
    }
    if (incoming == "p")
    {
      toSend = String(posDegrees);
    }
    if (incoming == "+")
    {
      PWM = PWM + 10;
      f_forward();
      toSend = String(PWM);
    }
    if (incoming == "-")
    {
      PWM = PWM - 10;
      f_forward();
      toSend = String(PWM);
    }
    if (incoming == "r")
    {
      PWM = 10;
      toSend = String(PWM);
    }
    if (incoming == "Wr")
    {
      toSend = String(W_encR);
      delay(50);
      toSend = String(tR);
      delay(50);
    }
    if (incoming == "Wl")
    {
      f_stop();
      toSend = String(W_encL);
      delay(50);
      toSend = String(tL);
      delay(50);
    }

    incoming = "";
    in = false;
  }

  encoder();
}

//*****************************
//*******  OTHER CORE  ********
//*****************************

void codeForTask1(void *parameter)
{

  for (;;)
  {

    while (SerialBT.available())
    {
      int letra = SerialBT.read();
      incoming += (char)letra;
    }

    if (incoming != "")
    {
      Serial.println(incoming);
      in = true;
    }

    if (toSend != "")
    {
      SerialBT.println(toSend);
      Serial.println(toSend);
      toSend = "";
    }

    vTaskDelay(2);
  }
}

//*****************************
//*******   FUNTIONS   ********
//*****************************

void move(int Dir_R[2], int Dir_L[2], int PWM_R, int PWM_L)
{
  digitalWrite(rigth_Dir_1, Dir_R[0]);
  digitalWrite(rigth_Dir_2, Dir_R[1]);
  digitalWrite(left_Dir_1, Dir_L[0]);
  digitalWrite(left_Dir_2, Dir_L[1]);
  ledcWrite(channelD, PWM_R); // Analog output - rigth
  ledcWrite(channelI, PWM_L); // Analog output - left
}

void f_move_R(int Dir_R[2], int PWM_R)
{
  digitalWrite(rigth_Dir_1, Dir_R[0]);
  digitalWrite(rigth_Dir_2, Dir_R[1]);
  ledcWrite(channelD, PWM_R); // Analog output - rigth
}

void f_move_L(int Dir_L[2], int PWM_L)
{
  digitalWrite(left_Dir_1, Dir_L[0]);
  digitalWrite(left_Dir_2, Dir_L[1]);
  ledcWrite(channelI, PWM_L); // Analog output - left
}

void f_forward()
{
  move(forward, forward, PWM, PWM);
}

void f_back()
{
  move(back, back, PWM, PWM);
}

void f_right()
{
  move(forward, forward, PWM, PWM / 2);
}

void f_left()
{
  move(forward, forward, PWM / 2, PWM);
}

void f_stop()
{
  move(stop, stop, 0, 0);
}

void clear()
{
  Serial.write(27);
  Serial.print("[2J");
  Serial.write(27);
  Serial.print("[H");
}

void encoder()
{
  enc_LA = digitalRead(left_A);
  enc_LB = digitalRead(left_B);
  enc_RA = digitalRead(rigth_A);
  enc_RB = digitalRead(rigth_B);

  if (enc_antLA != enc_LA || enc_antLB != enc_LB)
  {
    W_encL = processEnc(enc_antLA, enc_LA, enc_antLB, enc_LB, now_microSegL);
    contL++;

    now_microSegL = micros();
    tL = now_microSegL;
    enc_antLA = enc_LA;
    enc_antLB = enc_LB;
  }
  else if (W_encL != 0)
  {
    if (micros() - now_microSegL > 100000)
    {
      W_encL = 0;
      tL = now_microSegL;
      now_microSegL = micros();
      contL++;
    }
  }

  if (enc_antRA != enc_RA || enc_antRB != enc_RB)
  {
    W_encR = processEnc(enc_antRA, enc_RA, enc_antRB, enc_RB, now_microSegR);
    contR++;
    now_microSegR = micros();
    tR = now_microSegR;
    enc_antRA = enc_RA;
    enc_antRB = enc_RB;
  }
  else if (W_encR != 0)
  {
    if (micros() - now_microSegR > 100000)
    {
      W_encR = 0;
      now_microSegR = micros();
      tR = now_microSegR;
      contR++;
    }
  }
}

float processEnc(bool enc_antA, bool encA, bool enc_antB, bool encB, int now_microSeg)
{
  int dT = micros() - now_microSeg;
  for (size_t i = 1; i < 5; i++)
  {
    if (enc_antA == mov_back[i - 1][0] && encA == mov_back[i][0] && enc_antB == mov_back[i - 1][1] && encB == mov_back[i][1])
    {
      dT = -dT;
    }
    if (enc_antA == mov_forward[i - 1][0] && encA == mov_forward[i][0] && enc_antB == mov_forward[i - 1][1] && encB == mov_forward[i][1])
    {
      dT = dT;
    }
  }
  float velW = 1666.666 / ((float)dT);
  return velW;
}
