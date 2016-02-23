#include <AFMotor.h>

AF_DCMotor leftMotor(1);
AF_DCMotor rightMotor(2);

#define M1_MAX_SPEED 255
#define M2_MAX_SPEED 255
#define M1_DEFAULT_SPEED 125
#define M2_DEFAULT_SPEED 125

#define NUM_SENSORS 5
#define BUFFER_LEN 3
#define CAL_DARK A5
#define CAL_LIGHT A5
#define D_RANGE 20
#define P_SCALE 100


unsigned buffer[NUM_SENSORS];
unsigned current[NUM_SENSORS];
unsigned last[NUM_SENSORS];
unsigned base_dark[NUM_SENSORS];
unsigned base_light[NUM_SENSORS];
int error, deriv, last_err;
long integ;
char i_buffer;
float kp = 0.17;
float kd = 1.1;
float ki = 0.05;
long time, terr = 0;
char run = 20;
int tmax = 10000;


void calibrate()
{
  digitalWrite(CAL_LIGHT, HIGH);
  delay(3000);
  bool detected;
  while (!read_sensors(0, detected));
  for (unsigned i = 0; i < NUM_SENSORS; ++i)
  {
    base_light[i] = current[i];
  }
  
  digitalWrite(CAL_DARK, LOW);
  delay(3000);
  while (!read_sensors(0, detected));
  for (unsigned i = 0; i < NUM_SENSORS; ++i)
  {
    base_dark[i] = current[i];
  }
}

int map(int v, int s1, int s2, int d1, int d2)
{
  float ratio = float(d2 - d1) / float(s2 - s1);
  v = int((s2 - v) * ratio) + d1;
  if (v < d1) v = d1;
  if (v > d2) v = d2;
  return v;
}

bool read_sensors(bool adj, bool& detected)
{
  bool new_read = !(i_buffer < 8);
  unsigned long avg = 0;
  unsigned sum = 0;
  if (new_read) detected = 0;
  for (unsigned i = 0; i < NUM_SENSORS; ++i)
  {
    if (new_read)
    {
      i_buffer = 0;
      last[i] = current[i];
      current[i] = (buffer[i] >> 4);
      if (adj)
      {
        current[i] = map(current[i], base_dark[i], base_light[i], 0, D_RANGE);
      }
      if (current[i] > D_RANGE / 4) detected = 1;
      sum += current[i];
      avg += long(current[i]) * (i * P_SCALE);
      buffer[i] = 0;
    }
    buffer[i] += analogRead(i);
  }
  if (detected)
  {
    last_err = error;
    error = (avg / sum) - ((NUM_SENSORS - 1) * P_SCALE) / 2;
    integ += error;
    terr += abs(error);
  }
  else integ = 0;
  
  ++i_buffer;
  return new_read;
}

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A5, OUTPUT);
  set_motors(0,0);
  error = integ = i_buffer = 0;
  calibrate();
  for (unsigned i = 0; i < NUM_SENSORS; ++i)
  {
    Serial.print(base_light[i]);
    Serial.print("\t");
  }
  Serial.print("\n");
  delay(3000);
  time = millis();
  
}

int lastError = 0;

void loop() {

 bool detected = 0;
 if (read_sensors(1, detected))
 {
    deriv = error - last_err;
    float speed = (kp * float(error) + kd * float(deriv) + ki * float(integ));
    int speed_right = int(0.75 * float(127 - speed));
    int speed_left = int(0.75 * float(127 + speed));
    if (error < -30 && detected) speed_left = 0;
    if (error > 30 && detected) speed_right = 0;
    set_motors(speed_left,speed_right);
 }
}

void set_motors(int motor1speed, int motor2speed)
{
  if (motor1speed > M1_MAX_SPEED ) motor1speed = M1_MAX_SPEED; // limit top speed
  if (motor2speed > M2_MAX_SPEED ) motor2speed = M2_MAX_SPEED; // limit top speed
  if (motor1speed < 0) motor1speed = 0; // keep motor above 0
  if (motor2speed < 0) motor2speed = 0; // keep motor speed above 0
  rightMotor.setSpeed(motor1speed);     // set motor speed
  leftMotor.setSpeed(motor2speed);     // set motor speed
  rightMotor.run(FORWARD);  
  leftMotor.run(BACKWARD);
}

