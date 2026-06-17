#include <Arduino.h>

const int LDR_LEFT  = 34; // left photoresistor
const int LDR_RIGHT = 35;  // right photoresistor

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;

// left motors pins
const int L1 = 33;
const int L2 = 25;
const int L3 = 26;
const int L4 = 27;

// right motors pins
const int R1 = 21;
const int R2 = 19;
const int R3 = 17;
const int R4 = 16;

// ultrasonic sensor parameters
int OBS_DISTANCE = 20;
const unsigned long ECHO_TIMEOUT = 25000;

// PD control tuning parameters
int v_0 = 100;
float Kp = 0.10f;
float Kd = 0.65f;

float error_prev = 0.0f;
unsigned long last_time = 0;

// PWM
const int freq = 1000;
const int res  = 8;

// channels
const int chL1 = 0;
const int chL2 = 1;
const int chL3 = 2;
const int chL4 = 3;
const int chR1 = 4;
const int chR2 = 5;
const int chR3 = 6;
const int chR4 = 7;

// function declaration at the top of the code
long get_distance();
void left_motor(int speed);
void right_motor(int speed);
void motor_stop();
void PD(int leftVal, int rightVal);
int read_LDR(int pin);

// setup code
void setup()
{
  Serial.begin(115200);

  pinMode(LDR_LEFT, INPUT);
  pinMode(LDR_RIGHT, INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // pwm setup
  ledcSetup(chL1, freq, res); ledcAttachPin(L1, chL1);
  ledcSetup(chL2, freq, res); ledcAttachPin(L2, chL2);
  ledcSetup(chL3, freq, res); ledcAttachPin(L3, chL3);
  ledcSetup(chL4, freq, res); ledcAttachPin(L4, chL4);

  ledcSetup(chR1, freq, res); ledcAttachPin(R1, chR1);
  ledcSetup(chR2, freq, res); ledcAttachPin(R2, chR2);
  ledcSetup(chR3, freq, res); ledcAttachPin(R3, chR3);
  ledcSetup(chR4, freq, res); ledcAttachPin(R4, chR4);

  last_time = millis();
  motor_stop();
}

// loop code
void loop()
{
  int leftVal  = read_LDR(LDR_LEFT);
  int rightVal = read_LDR(LDR_RIGHT);
  long distance =get_distance();
  // printing information into a serial monitor
  Serial.print("L:");
  Serial.print(leftVal);
  Serial.print(" R:");
  Serial.print(rightVal);
  Serial.print(" D:");
  Serial.println(distance);

  // obstcle avoidance algorithm
if (distance > 0 && distance < OBS_DISTANCE)
  {
    left_motor(-v_0);
    right_motor(-v_0);
    delay(300);
    motor_stop();
    error_prev = 0;
    last_time = millis();
    delay(200);
    return;
  }
  PD(leftVal, rightVal);
  delay(20);
}

// smoothing photoresistor sensing Moving Average filtering
int read_LDR(int pin) {
  long sum = 0;
  for (int i = 0; i < 5; i++)
  {
    sum += analogRead(pin);
  }
  return sum / 5;
}

// code for PD control
void PD(int leftVal, int rightVal) {
  unsigned long now = millis();
  float dt = (now - last_time);
  if (dt <= 0) 
    dt = 1; // no division by 0!!!
  
  int raw_error = rightVal - leftVal;
  float error = (float)raw_error;

  // gain scheduling (GS)
  float GS = 1.0f;
  if (abs(raw_error) < 200) {
    GS = 1.5f;
  }

  float derivative = (error - error_prev) / dt;
  float u_gs = GS * (Kp * error + Kd * derivative);

  int speed_left  = v_0 - u_gs;
  int speed_right = v_0 + u_gs;

  speed_left  = constrain(speed_left,  -255, 255);
  speed_right = constrain(speed_right, -255, 255);

  left_motor(speed_left);
  right_motor(speed_right);

  error_prev = error;
  last_time = now;
}


// ultrasonic
const long NO_ECHO = -1;
long get_distance()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT);

  if (duration == 0) 
    return NO_ECHO;

  return duration * 0.034 / 2; // duration * speed of sound /2
}

// motor motion control
void left_motor(int speed) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);

  if (speed > 0) {
    ledcWrite(chL1, pwm);
    ledcWrite(chL2, 0);
    ledcWrite(chL3, 0);
    ledcWrite(chL4, pwm);
  }
  else if (speed < 0) {
    ledcWrite(chL1, 0);
    ledcWrite(chL2, pwm);
    ledcWrite(chL3, pwm);
    ledcWrite(chL4, 0);
  }
  else {
    ledcWrite(chL1, 0);
    ledcWrite(chL2, 0);
    ledcWrite(chL3, 0);
    ledcWrite(chL4, 0);
  }
}

void right_motor(int speed) {
  speed = constrain(speed, -255, 255);
  int pwm = abs(speed);
  if (speed > 0) {
    ledcWrite(chR1, 0);
    ledcWrite(chR2, pwm);
    ledcWrite(chR3, 0);
    ledcWrite(chR4, pwm);
  }
  else if (speed < 0) {
    ledcWrite(chR1, pwm);
    ledcWrite(chR2, 0);
    ledcWrite(chR3, pwm);
    ledcWrite(chR4, 0);
  }
  else {
    ledcWrite(chR1, 0);
    ledcWrite(chR2, 0);
    ledcWrite(chR3, 0);
    ledcWrite(chR4, 0);
  }
}

void motor_stop() {
  ledcWrite(chL1, 0);
  ledcWrite(chL2, 0);
  ledcWrite(chL3, 0);
  ledcWrite(chL4, 0);

  ledcWrite(chR1, 0);
  ledcWrite(chR2, 0);
  ledcWrite(chR3, 0);
  ledcWrite(chR4, 0);
}