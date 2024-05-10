#include <ElementStorage.h>
#include <BasicLinearAlgebra.h>

#define WHEEL_L 8
#define WHEEL_RADIUS 3
#define DIST_UNIT 1

#define DETECTOR_PIN A0
#define BUZZER_PIN 11
#define LEFT_MOTOR_PIN 4
#define RIGHT_MOTOR_PIN 7


using namespace BLA;

Matrix<3, 3> R(double th);
Matrix<4, 1> inverse_kinematic(Matrix<3, 1> vel, Matrix<8, 3> A, Matrix<8, 4> B);
int map_wheel_vel(double value);
float sweep_detector(unsigned long threshold);
void move_robot(Matrix<3, 1> vel, int duration);
void stop_robot(int duration);
void move_robot_circular(double r, double turn_angle, int step_count, int duration);

Matrix<4, 1> wheel_vel;
Matrix<8, 3> A;
Matrix<8, 4> B;
Matrix<3, 1> vel;
int wait_time = 800;
const float RC = 0.3;
float previousFilteredValue = 0;

int count = 0;
volatile int risingEdgeCount = 0;
double curTime = millis();
int avg = 0;
float xn1 = 0;
float yn1 = 0;
unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
const long interval1 = 300;  // Interval for function 1 (in milliseconds)
const long interval2 = 10;   // Interval for function 2 (in milliseconds)
bool changeFlag = false;
bool initFlag = false;
unsigned long threshold = 348;
bool dirFlag = false; // False - left; True - right
int dirCount = 0;

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  pinMode(3, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // The constraint matrices
  A = {sin(PI / 4 + PI / 4)   , -cos(PI / 4 + PI / 4)    , -WHEEL_L * cos(PI / 4),
       sin(3 * PI / 4 - PI / 4) , -cos(3 * PI / 4 - PI / 4)  , -WHEEL_L * cos(-PI / 4),
       sin(-3 * PI / 4 + -3 * PI / 4), -cos(-3 * PI / 4 + -3 * PI / 4) , -WHEEL_L * cos(-3 * PI / 4),
       sin(-PI / 4 + 3 * PI / 4) , -cos(-PI / 4 + 3 * PI / 4)  , -WHEEL_L * cos(3 * PI / 4),

       cos(PI / 4 + PI / 4)   ,  sin(PI / 4 + PI / 4)    ,  WHEEL_L * sin(PI / 4),
       cos(3 * PI / 4 - PI / 4) ,  sin(3 * PI / 4 - PI / 4)  ,  WHEEL_L * sin(-PI / 4),
       cos(-3 * PI / 4 + -3 * PI / 4),  sin(-3 * PI / 4 + -3 * PI / 4) ,  WHEEL_L * sin(-3 * PI / 4),
       cos(-PI / 4 + 3 * PI / 4) ,  sin(-PI / 4 + 3 * PI / 4)  ,  WHEEL_L * sin(3 * PI / 4)
      };

  B = {WHEEL_RADIUS, 0, 0, 0,
       0, WHEEL_RADIUS, 0, 0,
       0, 0, WHEEL_RADIUS, 0,
       0, 0, 0, WHEEL_RADIUS,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0,
       0, 0, 0, 0
      };
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis1 >= interval1) {
    previousMillis1 = currentMillis;
    if (dirCount <= 2) {
      move_robot_circular(0.3, PI / 6, 1, wait_time);
      dirCount += 1;
    } else if (dirCount <= 5){
      move_robot_circular(0.3, -PI / 6, 1, wait_time);
      dirCount += 1;
    } else {
      dirCount = 0;
    }
  }

  if (currentMillis - previousMillis2 >= interval2) {
    previousMillis2 = currentMillis;
    if (!initFlag) {
      for (int i = 0; i < 500; i++) threshold = sweep_detector(threshold);
      initFlag = true;
    } else
      sweep_detector(threshold);
  }
}


// ------------------- FUNCTIONS -------------------
void move_robot_circular(double r, double turn_angle, int step_count, int duration) {
  turn_angle *= -1;
  double step_angle = turn_angle / step_count;
  double origin_x = r * cos(turn_angle);
  double origin_y = r * sin(turn_angle);
  double phi = 0;

  for (int i = 0; i < step_count; i++) {
    vel = {origin_x - cos( turn_angle + step_angle * (i + 1) ) * r, origin_y + sin( turn_angle + step_angle * (i + 1) ) * r, step_angle*(i + 1)};
    // Calculate the wheel velocity
    wheel_vel =  inverse_kinematic(vel, A, B);
    // Move the motors
    digitalWrite(LEFT_MOTOR_PIN ,  map_wheel_vel(wheel_vel(0, 1)));
    digitalWrite(RIGHT_MOTOR_PIN,  map_wheel_vel(wheel_vel(0, 3)));
  }
}

void move_robot(Matrix<3, 1> vel, int duration) {
  // Calculate the wheel velocity
  wheel_vel =  inverse_kinematic(vel, A, B);

  Serial << "Body Vel: " << vel << '\n';
  Serial << "All Wheel: " << wheel_vel << '\n';
  Serial << "Left Wheel: " << wheel_vel(0, 1) << '\n';
  Serial << "Right Wheel: " << wheel_vel(0, 3) << '\n';

  // Move the motors
  digitalWrite(LEFT_MOTOR_PIN ,  map_wheel_vel(wheel_vel(0, 1)));
  digitalWrite(RIGHT_MOTOR_PIN,  map_wheel_vel(wheel_vel(0, 3)));
  delay(duration);
}

void stop_robot(int duration) {
  digitalWrite(LEFT_MOTOR_PIN ,  LOW);
  digitalWrite(RIGHT_MOTOR_PIN,  LOW);
  delay(duration);
}

float sweep_detector(unsigned long threshold) {
  // This function sweeps the car to read the metal detector values.
  // It returns the detected angle in degree 0 - 90 - 180
  // If no metal detected, it returns -1
  avg = 0;
  for (int i = 0; i < 10; i++) {
    avg += pulseIn(3, HIGH);
  }
  avg = avg / 10;
  float xn = avg;

  float yn = 0.93908194 * yn1 + 0.03045903 * xn + 0.03045903 * xn1;

  xn1 = xn;
  yn1 = yn;
  if (initFlag) {
    if (threshold + 9 < yn) {
      tone(BUZZER_PIN, 1000);
    } else {
      noTone(BUZZER_PIN);
    }
  }
  Serial.print(threshold);
  Serial.print(',');
  Serial.print(yn);
  Serial.print(',');
  Serial.println();
  return yn;
}

int map_wheel_vel(double value) {
  // Maps the wheel velocity value to the motor values.
  return (value > 0) ? HIGH : LOW;
}

Matrix<4, 1> inverse_kinematic(Matrix<3, 1> vel, Matrix<8, 3> A, Matrix<8, 4> B) {
  // Given the desired robot velocity it calculates the wheel velocities.
  //FL RL RR FR
  Matrix<4, 1> wheel_vel = Inverse(~B * B) * ~B * A * vel;
  return wheel_vel;
}
