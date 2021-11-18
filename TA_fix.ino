#include <stdint.h>

#define MIN_ACCEL_X 200
#define MAX_ACCEL_X 200
#define MIN_ACCEL_Y 100
#define MAX_ACCEL_Y 200
#define MAX_SPEED_X 18000
#define MAX_SPEED_Y 17000
#define ACCEL_RAMP_MIN 2500
#define X_AXIS_STEPS_PER_UNIT 19
#define Y_AXIS_STEPS_PER_UNIT 19
#define ROBOT_MIN_X 70
#define ROBOT_MIN_Y 0
#define ROBOT_MAX_X 530
#define ROBOT_MAX_Y 400
#define ROBOT_CENTER_X 300
#define ROBOT_CENTER_Y 500
#define ROBOT_INITIAL_POSITION_X 300
#define ROBOT_INITIAL_POSITION_Y 0
#define CAM_PIX_WIDTH 320
#define CAM_PIX_HEIGHT 240
#define CAM_PIX_CENTER_X 160
#define CAM_PIX_CENTER_Y 120
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define ZERO_SPEED 65535

// Variable definitions
int puckXold;
int puckYold;
int robotXold;
int robotYold;
float Kp;
float Ki;
float Kd;
float Kpredictor;
int Prop;
int Int;
int Der;
int lastError;
int error;
float total;
float totalX;
float vektorX;
float predictor;
float deltaX;
float waktuframe;
unsigned long sekarang;
unsigned long lalu;
unsigned long elapsedTime;
float A, CA, D;
bool L, M, S; //ERROR
bool N, Z, P; // DE
float E, DE, lastE;
float uE[] = {0, 0, 0, 0, 0};
float uDE[] = {0, 0, 0, 0, 0}; //membership
float z1, z2, z3, z4, z5, z6, z7, z8, z9;
float w1, w2, w3, w4, w5, w6, w7, w8, w9;
float out;

long loop_counter;
long timer_old;
long timer_packet_old;
long timer_value;
int debug_counter;
uint32_t micros_old;

volatile int16_t position_x;  // This variables are modified inside the Timer interrupts
volatile int16_t position_y;
int16_t speed_x;
int16_t speed_y;
int8_t dir_x;     //(dir=1 positive, dir=-1 negative)
int8_t dir_y;
int16_t target_position_x;
int16_t target_position_y;
int16_t target_speed_x;
int16_t target_speed_y;
int16_t max_acceleration_x = MAX_ACCEL_X;
int16_t max_acceleration_y = MAX_ACCEL_Y;
int16_t acceleration_x = MAX_ACCEL_X;
int16_t acceleration_y = MAX_ACCEL_Y;
int16_t accel_ramp = ACCEL_RAMP_MIN;
int16_t pos_stop_x;
int16_t pos_stop_y;
int puckPixX;
int puckPixY;
int robotPixX;
int robotPixY;
int cam_center_x;
int cam_center_y;
float cam_rotation;

int SBuffer[6];
uint8_t readStatus;
uint8_t readCounter;
uint8_t newPacket;
uint16_t com_pos_x;
uint16_t com_pos_y;
uint16_t com_speed_x;
uint16_t com_speed_y;
uint16_t target_x_mm;
uint16_t target_y_mm;

int sign(int val)
{
  if (val < 0)
    return (-1);
  else
    return (1);
}

uint16_t extractParamInt(uint8_t pos) {
  union {
    int Buff[2];
    uint16_t d;
  }
  u;

  u.Buff[0] = SBuffer[pos];
  u.Buff[1] = SBuffer[pos + 1];
  return (u.d);
}

void packetRead()
{
  unsigned char i;
  if (Serial.available() > 0) {
    for (i = 3; i > 0; i--) {
      SBuffer[i] = SBuffer[i - 1];
    }
    SBuffer[0] = Serial.parseInt();
    if ((SBuffer[0] == 1000) && (SBuffer[1] == 1000))
    {
      if (readStatus == 0)
      {
        readStatus = 1;
        readCounter = 4;
        puckXold = puckPixX;
        puckYold = puckPixY;
        robotXold = robotPixX;
        robotYold = robotPixY;
      }
      else
      {
        readStatus = 1;
        readCounter = 4;
        puckXold = puckPixX;
        puckYold = puckPixY;
        robotXold = robotPixX;
        robotYold = robotPixY;
      }
      return;
    }
    else if (readStatus == 1)
    {
      readCounter--;
      if (readCounter <= 0)
      {
        puckPixY = extractParamInt(3);
        puckPixX = extractParamInt(2);
        robotPixY = extractParamInt(1);
        robotPixX = extractParamInt(0);
        readStatus = 0;
        newPacket = 1;
      }
    }
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (dir_x == 0)
    return;
  SET(PORTF, 0); // STEP X-AXIS
  position_x += dir_x;
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
  CLR(PORTF, 0);
}

ISR(TIMER3_COMPA_vect)
{
  if (dir_y == 0)
    return;
  SET(PORTF, 6); // STEP Y-AXIS (Y-left)
  SET(PORTL, 3); // STEP Z-AXIS (Y-right)
  position_y += dir_y;
  __asm__ __volatile__ (
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop" "\n\t"
    "nop");
  CLR(PORTF, 6);
  CLR(PORTL, 3);
}

void positionControl()
{
  //int16_t pos_stop;
  int32_t temp;
  uint32_t timer;
  int16_t dt;

  SET(PORTF, 3);
  timer = micros();
  dt = constrain(timer - micros_old, 0, 2000);
  micros_old = timer;
  acceleration_x = map(abs(speed_x), 0, accel_ramp, MIN_ACCEL_X, max_acceleration_x);
  acceleration_x = constrain(acceleration_x, MIN_ACCEL_X, max_acceleration_x);
  acceleration_y = map(abs(speed_y), 0, accel_ramp, MIN_ACCEL_Y, max_acceleration_y);
  acceleration_y = constrain(acceleration_y, MIN_ACCEL_Y, max_acceleration_y);

  // X AXIS
  temp = (long)speed_x * speed_x;
  temp = temp / (1800 * (long)acceleration_x);
  pos_stop_x = position_x + sign(speed_x) * temp;
  if (target_position_x > position_x)
  {
    if (pos_stop_x >= target_position_x)
      setMotorXSpeed(0, dt);
    else
      setMotorXSpeed(target_speed_x, dt);
  }
  else
  {
    if (pos_stop_x <= target_position_x)
      setMotorXSpeed(0, dt);
    else
      setMotorXSpeed(-target_speed_x, dt);
  }

  // Y AXIS
  temp = (long)speed_y * speed_y;
  temp = temp / (1800 * (long)acceleration_y);
  pos_stop_y = position_y + sign(speed_y) * temp;
  if (target_position_y > position_y)
  {
    if (pos_stop_y >= target_position_y)
      setMotorYSpeed(0, dt);
    else
      setMotorYSpeed(target_speed_y, dt);
  }
  else
  {
    if (pos_stop_y <= target_position_y)
      setMotorYSpeed(0, dt);
    else
      setMotorYSpeed(-target_speed_y, dt);
  }
  CLR(PORTF, 3);
}

void setMotorXSpeed(int16_t tspeed, int16_t dt)
{
  long timer_period;
  int16_t accel;
  if (tspeed > MAX_SPEED_X)
    tspeed = MAX_SPEED_X;
  else if (tspeed < -MAX_SPEED_X)
    tspeed = -MAX_SPEED_X;
  accel = ((long)acceleration_x * dt) / 1000;
  if (((long)tspeed - speed_x) > accel)
    speed_x += accel;
  else if (((long)speed_x - tspeed) > accel)
    speed_x -= accel;
  else
    speed_x = tspeed;
  if ((speed_x == 0) && (dir_x != 0))
    dir_x = 0;
  else if ((speed_x > 0) && (dir_x != 1))
  {
#ifdef INVERT_X_AXIS
    CLR(PORTF, 1);  // X-DIR
#else
    SET(PORTF, 1);
#endif
    dir_x = 1;
  }
  else if ((speed_x < 0) && (dir_x != -1))
  {
#ifdef INVERT_X_AXIS
    SET(PORTF, 1);
#else
    CLR(PORTF, 1);
#endif
    dir_x = -1;
  }

  if (speed_x == 0)
    timer_period = ZERO_SPEED;
  else if (speed_x > 0)
    timer_period = 2000000 / speed_x; // 2Mhz timer
  else
    timer_period = 2000000 / -speed_x;

  if (timer_period > 65535)
    timer_period = ZERO_SPEED;

  OCR1A = timer_period;
  // cek apakah butuh reset timer?
  if (TCNT1 > OCR1A)
    TCNT1 = 0;
}

void setMotorYSpeed(int16_t tspeed, int16_t dt)
{
  long timer_period;
  int16_t accel;
  if (tspeed > MAX_SPEED_Y)
    tspeed = MAX_SPEED_Y;
  else if (tspeed < -MAX_SPEED_Y)
    tspeed = -MAX_SPEED_Y;
  accel = ((long)acceleration_y * dt) / 1000;
  if (((long)tspeed - speed_y) > accel)
    speed_y += accel;
  else if (((long)speed_y - tspeed) > accel)
    speed_y -= accel;
  else
    speed_y = tspeed;
  if ((speed_y == 0) && (dir_y != 0))
    dir_y = 0;
  else if ((speed_y > 0) && (dir_y != 1))
  {
#ifdef INVERT_Y_AXIS // Y-DIR (Y-left)
    CLR(PORTF, 7);
#else
    SET(PORTF, 7);
#endif

#ifdef INVERT_Z_AXIS  // Z-DIR (Y-right)
    CLR(PORTL, 1);
#else
    SET(PORTL, 1);
#endif
    dir_y = 1;
  }
  else if ((speed_y < 0) && (dir_y != -1))
  {
#ifdef INVERT_Y_AXIS  // Y-DIR (Y-left)
    SET(PORTF, 7);
#else
    CLR(PORTF, 7);
#endif

#ifdef INVERT_Z_AXIS  // Z-DIR (Y-right)
    SET(PORTL, 1);
#else
    CLR(PORTL, 1);
#endif
    dir_y = -1;
  }

  if (speed_y == 0)
    timer_period = ZERO_SPEED;
  else if (speed_y > 0)
    timer_period = 2000000 / speed_y;
  else
    timer_period = 2000000 / -speed_y;

  if (timer_period > 65535)
    timer_period = ZERO_SPEED;
  OCR3A = timer_period;
  // Check  kalo butuh reset timer
  if (TCNT3 > OCR3A)
    TCNT3 = 0;
}

void setSpeedS(int target_sx, int target_sy)
{
  target_sx = constrain(target_sx, 0, MAX_SPEED_X);
  target_sy = constrain(target_sy, 0, MAX_SPEED_Y);
  target_speed_x = target_sx;
  target_speed_y = target_sy;
}

void setPosition(int target_x_mm_new, int target_y_mm_new)
{
  target_x_mm = constrain(target_x_mm_new, ROBOT_MIN_X, ROBOT_MAX_X);
  target_y_mm = constrain(target_y_mm_new, ROBOT_MIN_Y, ROBOT_MAX_Y);
  target_position_x = target_x_mm * X_AXIS_STEPS_PER_UNIT;
  target_position_y = target_y_mm * Y_AXIS_STEPS_PER_UNIT;
}

void setup() {
  pinMode(38, OUTPUT); // ENABLE MOTOR
  pinMode(A0, OUTPUT); // STEP MOTOR
  pinMode(A1, OUTPUT); // DIR MOTOR
  // Y_AXIS (Y-LEFT)
  pinMode(A2, OUTPUT); // ENABLE MOTOR
  pinMode(A6, OUTPUT); // STEP MOTOR
  pinMode(A7, OUTPUT); // DIR MOTOR
  // Z_AXIS (Y-RIGHT)
  pinMode(A8, OUTPUT); // ENABLE MOTOR
  pinMode(46, OUTPUT); // STEP MOTOR
  pinMode(48, OUTPUT); // DIR MOTOR

  pinMode(A3, OUTPUT);

  pinMode(19, INPUT); // RX1 Serial Port 1
  pinMode(18, OUTPUT); // TX1

  //LEDS
  pinMode(13, OUTPUT);

  // Disable Motors
  digitalWrite(38, HIGH);
  digitalWrite(A2, HIGH);
  digitalWrite(A8, HIGH);

  Serial.begin(115200);

  //setup PID
  Kp = 0.135;
  Ki = 0;
  Kd = 0.0000000008;
  Kpredictor = 0;
  puckXold = 101;
  puckPixX = 101;

  //setup FLC variables
  E = 0;
  DE = 0;
  lastE = 0;

  L = 0;
  M = 0;
  S = 0;
  N = 0;
  Z = 0;
  P = 0;

  cam_center_x = CAM_PIX_CENTER_X;
  cam_center_y = CAM_PIX_CENTER_Y;

  //LED blink
  for (uint8_t k = 0; k < 4; k++)
  {
    digitalWrite(13, HIGH);
    delay(300);
    digitalWrite(13, LOW);
    delay(300);
  }

  // pake TIMER 1 untuk X, TIMER 3 untuk Y
  TCCR1B &= ~(1 << WGM13);
  TCCR1B |=  (1 << WGM12);
  TCCR1A &= ~(1 << WGM11);
  TCCR1A &= ~(1 << WGM10);
  TCCR1A &= ~(3 << COM1A0);
  TCCR1A &= ~(3 << COM1B0);
  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);
  OCR1A = ZERO_SPEED;
  dir_x = 0;
  TCNT1 = 0;
  TCCR3B &= ~(1 << WGM13);
  TCCR3B |=  (1 << WGM12);
  TCCR3A &= ~(1 << WGM11);
  TCCR3A &= ~(1 << WGM10);
  TCCR3A &= ~(3 << COM1A0);
  TCCR3A &= ~(3 << COM1B0);
  TCCR3B = (TCCR3B & ~(0x07 << CS10)) | (2 << CS10);
  OCR3A = ZERO_SPEED;
  dir_y = 0;
  TCNT3 = 0;
  position_x = ROBOT_INITIAL_POSITION_X * X_AXIS_STEPS_PER_UNIT;
  position_y = ROBOT_INITIAL_POSITION_Y * Y_AXIS_STEPS_PER_UNIT;
  delay(1000);
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
  TIMSK3 |= (1 << OCIE1A); // Enable Timer1 interrupt
  digitalWrite(38, LOW);  // X-axis
  digitalWrite(A2, LOW);  // Y-axis left
  digitalWrite(A8, LOW);  // Z-axis (Y-axis right)
  com_pos_x = ROBOT_INITIAL_POSITION_X;
  com_pos_y = ROBOT_INITIAL_POSITION_Y;
  com_speed_x = MAX_SPEED_X;
  com_speed_y = MAX_SPEED_Y;
  setSpeedS(com_speed_x, com_speed_y);
  setPosition(com_pos_x, com_pos_y);
  timer_old = micros();
  timer_packet_old = timer_old;
  micros_old = timer_old;
  waktuframe = 12.5;
}

void loop() {
  timer_value = micros();
  if ((timer_value - timer_old) >= 1000) // 1Khz loop
  {
    timer_old = timer_value;
    packetRead();
    if (newPacket == 1) {
      newPacket = 0;
      pidX();
      flcY();
      setSpeedS(MAX_SPEED_X, MAX_SPEED_Y);
      setPosition(com_pos_x, com_pos_y);
    }
    positionControl();
  } // 1Khz loop
}

void pidX() {
  sekarang = millis();
  elapsedTime = (sekarang - lalu) / waktuframe;
  error = (puckPixX - robotPixX); //robot akan mengejar bola, satuan piksel
  Prop = error;
  Int = Int + (error * elapsedTime);
  Der = (error - lastError) / elapsedTime;
  lastError = error;
  total = (Prop * Kp + Int * Ki + Der * Kd)* (480 / 170);

  if (puckPixX - puckXold > 1 || puckPixX - puckXold < -1) {
    vektorX = ((puckPixX - puckXold) * (480 / 170));
  }
  else {
    vektorX = 0;
  }
  if (total > 0 && vektorX < 0 || total < 0 && vektorX > 0) {
    totalX = 0;
    total = 0;
    Prop = 0;
    Int = 0;
    Der = 0;
    lastError = 0;
    error = 0;
  }
  else {
    totalX = total;
  }
  predictor = Kpredictor * vektorX;
  lalu = sekarang;
  com_pos_x += totalX + predictor;
}

void flcY() {
  E = puckPixY - 176;
  DE = E - lastE;

  fuzzifikasi();
  rules();
  defuzzifikasi();

  lastE = E;
}

void fuzzifikasi() {
  Large();
  Medium();
  Small();
  Positive();
  Zero();
  Negative();
}

void rules() {
  //If (Error is Negatif) and (DeltaError is Negatif) then (Step is Negatif)
  w1 = min(uE[2], uDE[2]);
  z1 = 0;
  //If (Error is Negatif) and (DeltaError is Zero) then (Step is Negatif)
  w2 = min(uE[2], uDE[1]);
  z2 = 0;
  //If (Error is Negatif) and (DeltaError is Positif) then (Step is Negatif)
  w3 = min(uE[2], uDE[0]);
  z3 = 0;
  //If (Error is Zero) and (DeltaError is Negatif) then (Step is Negatif)
  w4 = min(uE[1], uDE[2]);
  z4 = 0;
  //If (Error is Zero) and (DeltaError is Zero) then (Step is Zero)
  w5 = min(uE[1], uDE[1]);
  z5 = 200;
  //If (Error is Zero) and (DeltaError is Positif) then (Step is Positif)
  w6 = min(uE[1], uDE[0]);
  z6 = 350;
  //If (Error is Positif) and (DeltaError is Negatif) then (Step is Positif)
  w7 = min(uE[0], uDE[2]);
  z7 = 350;
  //If (Error is Positif) and (DeltaError is Zero) then (Step is Positif)
  w8 = min(uE[0], uDE[1]);
  z8 = 350;
  //If (Error is Positif) and (DeltaError is Positif) then (Step is Positif)
  w9 = min(uE[0], uDE[0]);
  z9 = 350;
}

void defuzzifikasi() {
  float NUM = ((z1 * w1) + (z2 * w2) + (z3 * w3) + (z4 * w4) + (z5 * w5) + (z6 * w6) + (z7 * w7) + (z8 * w8) + (z9 * w9));
  float DEN = (w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9);
  out = NUM / DEN;
  com_pos_y = out;
}

unsigned char Large() {
  if (E <= 0) {
    uE[0] = 0;
    L = 0;
  } else if (E >= 87.5 && E <= 175) {
    uE[0] = (E - 87.5) / 87.5;
    L = 1;
  } else if (E >= 175) {
    uE[0] = 1;
    L = 1;
  }
  return uE[0];
}

unsigned char Medium() {
  if (E <= -87.5) {
    uE[1] = 0;
    M = 0;
  } else if (E >= -87.5 && E <= 0) {
    uE[1] = (E - (-87.5)) / 87.5;
    M = 1;
  } else if (E >= 0 && E <= 87.5) {
    uE[1] = (87.5 - E) / 87.5;
    M = 1;
  } else if (E >= 87.5) {
    uE[1] = 0;
    M = 0;
  }
  return uE[1];
}

unsigned char Small() {
  if (E <= -87.5) {
    uE[2] = 1;
    S = 1;
  } else if (E >= -87.5 && E <= 0) {
    uE[2] = (0 - E) / 87.5;
    S = 1;
  } else if (E >= 0) {
    uE[2] = 0;
    S = 0;
  }
  return uE[2];
}

unsigned char Positive() {
  if (DE <= 0) {
    uDE[0] = 0;
    P = 0;
  } else if (DE >= 0 && DE <= 5) {
    uDE[0] = (DE - 0) / 5;
    P = 1;
  } else if (DE >= 5) {
    uDE[0] = 1;
    P = 1;
  }
  return uDE[0];
}

unsigned char Zero() {
  if (DE <= -5) {
    uDE[1] = 0;
    Z = 0;
  } else if (DE >= -5 && DE <= 0) {
    uDE[1] = (DE + 5) / 5;
    Z = 1;
  } else if (DE >= 0 && DE <= 5) {
    uDE[1] = (5 - DE) / 5;
    Z = 1;
  } else if (DE >= 5) {
    uDE[1] = 0;
    Z = 0;
  }
  return uDE[1];
}

unsigned char Negative() {
  if (DE <= -5) {
    uDE[2] = 1;
    N = 1;
  } else if (DE >= -5 && DE <= 0) {
    uDE[2] = (0 - DE) / 5;
    N = 1;
  } else if (DE >= 0) {
    uDE[2] = 0;
    N = 0;
  }
  return uDE[2];
}
