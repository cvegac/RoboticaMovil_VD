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
long lastAction = 0;
long lastServo = 0;
long lastLaser = 0;
int count = 0;

// Communication
String accion = "";
String incoming = "";
String toSend = "";
bool in = false;

// Setting PWM properties
int PWM = 256;
const int freq = 100;
const int channelI = 8;
const int channelD = 9;
const int resolution = 10;

// MPU-6050
int16_t AcX, AcY, GyZ;

float Acc[2];
float Gy_Z;
float Ac_X;
float Ac_Y;
float Angle_Z;
float Vel_X;
float Vel_Y;
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

float Wp_encR;
float Wp_encL;

long prevTime;
float dt;

// Time between action (milliseconds)
int laserPeriod = 200;
int servoPeriod = 200;
int movilControlPeriod = 150;
int motorControlPeriod = 15;
int actionPeriod = 50;

/* Function declaration */
void codeForTask1(void *parameter);
void move(int Dir_R[2], int Dir_L[2], int PWM_R, int PWM_L);
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

  Serial.println("VL53L0X test");
  if (!lox.begin())
  {
    Serial.println(F("Error al iniciar VL53L0X"));
    while (1)
      ;
  }

  Serial.begin(1500000);
  clear();

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
  if (now - lastLaser > laserPeriod)
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
  if (now - lastLecture >= movilControlPeriod)
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
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true);
    GyZ = Wire.read() << 8 | Wire.read();

    Ac_X = AcX / A_R;
    Ac_Y = AcY / A_R;
    Gy_Z = GyZ / G_R;

    dt = (millis() - prevTime) / 1000.0;
    prevTime = millis();

    // Integrals to obtain moviment in X
    Vel_X = Vel_X + Ac_X * dt;
    Post_X = Post_X + Ac_X * dt;

    // Integrals to obtain moviment in Y
    Vel_Y = Vel_Y + Ac_Y * dt;
    Post_Y = Post_Y + Ac_Y * dt;

    // Integral to obtain YAW
    Angle_Z = Angle_Z + Gy_Z * dt - 0.012; // This 0.012 is for the Drift Error
  }

  if (now - lastAction > 5)
  {
    lastAction = now;
    if (count < 4)
    {
      f_stop();
    }
    else
    {
      if (accion == "B")
      {
        f_back();
      }
      if (accion == "R")
      {
        f_right();
      }
      if (accion == "L")
      {
        f_left();
      }
      if (accion == "F")
      {
        digitalWrite(led, HIGH);
        f_forward();
      }
      if (accion == "S")
      {
        digitalWrite(led, LOW);
        f_stop();
      }
      if (accion == "a")
      {
        toSend = String(Angle_Z);
      }
      if (accion == "d")
      {
        toSend = String(distance);
      }
      if (accion == "p")
      {
        toSend = String(posDegrees);
      }
      if (accion == "+")
      {
        PWM = PWM + 10;
        f_forward();
        toSend = String(PWM);
      }
      if (accion == "-")
      {
        PWM = PWM - 10;
        f_forward();
        toSend = String(PWM);
      }
      if (accion == "r")
      {
        PWM = 10;
        toSend = String(PWM);
      }
      accion = "";
    }
    count++;
  }

  if (in)
  {
    if (incoming == "W")
    {
    }
    if (incoming == "w")
    {
    }
    accion = incoming;
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
    Wp_encL = processEnc(enc_antLA, enc_LA, enc_antLB, enc_LB, now_microSegL);
    Serial.print("L: ");
    Serial.println(Wp_encL);
    now_microSegL = micros();
    enc_antLA = enc_LA;
    enc_antLB = enc_LB;
  }
  else if (Wp_encL != 0)
  {
    if (micros() - now_microSegL > 100000)
    {
      Wp_encL = 0;
      Serial.println("R: 0");
    }
  }

  if (enc_antRA != enc_RA || enc_antRB != enc_RB)
  {
    Wp_encR = processEnc(enc_antRA, enc_RA, enc_antRB, enc_RB, now_microSegR);
    Serial.print("R: ");
    Serial.println(Wp_encR);
    now_microSegR = micros();
    enc_antRA = enc_RA;
    enc_antRB = enc_RB;
  }
  else if (Wp_encR != 0)
  {
    if (micros() - now_microSegR > 100000)
    {
      Wp_encR = 0;
      Serial.println("R: 0");
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
