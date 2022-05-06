#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//State machine states when RUNNING
enum RUN_STATE {
  DETECT,
  FIND_FIRE,
  END,
};

enum FIRE_FIGHTING_STATE {
  FORWARD_DEFAULT,
  FORWARD_PASS,
  STRAFE_LEFT,
  STRAFE_RIGHT,
  EXTINGUISH,
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
const byte turret = 52;

//----Bluetooth Comms----
#define BLUETOOTH_RX 10
#define BLUETOOTH_TX 11
#define STARTUP_DELAY 5 // Seconds
#define LOOP_DELAY 100 // miliseconds
#define SAMPLE_DELAY 10 // miliseconds
//change serial output
#define OUTPUTMONITOR 0
#define OUTPUTPLOTTER 0
#define OUTPUTBLUETOOTHMONITOR 1
volatile int32_t Counter = 1;
SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);
//----Bluetooth Comms----

//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;
float Ultraduration;
float Ultradistance;
const float ultra_centre_offset = 11.0;
//----Ultrasound----

//----IR----
enum IR {
  FRONT_LEFT, // mid
  FRONT_RIGHT, // mid
  LEFT, // long
  RIGHT, // long
};

// Left front long range IR
const int IR_LONG_1 = A4;
float IR_LONG_1_DIST = 0;

// Left back long range IR
const int IR_LONG_2 = A5;
float IR_LONG_2_DIST = 0;

float IR_diff = 0;
float correction = 0;
float IR_wall_dist = 0;
float IR_Angle = 0;
const float IR_Between_Dist = 12;
const float long_centre_offset = 4.5;

// Back left mid range IR
const int IR_MID_1 = A6;
float IR_MID_1_DIST = 0;

// Back right mid range IR
const int IR_MID_2 = A7;
float IR_MID_2_DIST = 0;

float IR_mid_diff = 0;
float IR_mid_dist = 0;
const float mid_centre_offset = 7.0;
//----IR----

//----IR Kalman Filter----
float last_est[4] = {0, 0, 0, 0};
float last_var[4] = {999, 999, 999, 999};
float process_noise = 5;
float sensor_noise = 6;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----Phototransistor----
enum PT {
  PT1, // left most
  PT2, // left centre
  PT3, // right centre
  PT4, // right most
};

const int PT1_pin = A9;
const int PT2_pin = A10;
const int PT3_pin = A11;
const int PT4_pin = A12;

int PT1_reading = 0;
int PT2_reading = 0;
int PT3_reading = 0;
int PT4_reading = 0;
int PT_sum = 0;

const int detection_threshold = 100; // SUBJECT TO CHANGE
//----Phototransistor----

//----Obstacle Avoidance----
bool strafed_left = false;
bool strafed_right = false;
//----Obstacle Avoidance----

//----Fire Fighting----
const int mosfetPin = 45; // mosfet pin for fan

int numFires = 0;
bool fire_is_close = false;
bool fanAligned = false;
bool extinguished = false;
//----Fire Fighting----

//----MA Filter----
#define WINDOW_SIZE 13
int index[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float value[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float SUM[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO

float FRONT_LIR[WINDOW_SIZE];
float BACK_LIR[WINDOW_SIZE];
float LEFT_MIR[WINDOW_SIZE];
float RIGHT_MIR[WINDOW_SIZE];
float SONAR[6];
float LEFT_PT_1[WINDOW_SIZE];
float LEFT_PT_2[WINDOW_SIZE];
float RIGHT_PT_1[WINDOW_SIZE];
float RIGHT_PT_2[WINDOW_SIZE];

float averaged[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO, [6]-[9]PT
//----MA Filter----

//----Gyro----
const int gyroPin = A8;           //define the pin that gyro is connected
int gyroADC = 0;           // read out value of sensor
float gyroSupplyVoltage = 5;      // supply voltage for gyro
const float gyroMiddle = 505; // middle adc value of gyro
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 1;      // because of gyro drifting, defining rotation angular velocity less
float angularVelocity = 0;
// than this value will be ignored
int timeElapsed = 0;
int prev_gyroTime = 0;
int gyroTime = 0;
float gyroRate = 0;                      // read out value of sensor in voltage
float angleChange = 0;
float currentAngle = 0;               // current angle calculated by angular velocity integral on
float gyroCorrection = 0;
float Kp_gyro = 35;

float radiansAngle = 0;
//----Gyro----

//----Driving----
bool DRIVING = false;
bool accelerated = false;
const int forward_speed = 200;
const int strafe_speed = 150;
//----Driving----

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//----InverseKinematicValues----
float L = 007.620;
float l = 009.078;
float R_w = 002.54;

float velocity[3] = {0, 0, 0};
float max_velocity[3] = {20, 15, 1.5}; // cm/s
float ang_vel[4] = {0, 0, 0, 0};

float motor_speed_Value[4] = {0, 0, 0, 0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {0, 0, 0};
float currentTime, previousTime, elapsedTime;
float totalTime = 0;
float error[3] = {0, 0, 0};
float lastError[3] = {0, 0, 0};
float rateError[3] = {0, 0, 0};

float Pterm[3], Iterm[3], Dterm[3];

// PID VALUES FOR X, Y AND Z
float Kp[3] = {2.5, 2.2, 1.65};
float Ki[3] = {0.1, 0.03, 0.05};
float Kd[3] = {0, 0, 0};
//----PIDValues----

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  // The Trigger pin will tell the sensor to range find
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  pinMode(mosfetPin, OUTPUT);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);

  //Serial.begin(115200);

  BluetoothSerial.begin(115200);
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}


STATE initialising() {
  enable_motors();
  return RUNNING;
}

STATE running() {

  static RUN_STATE running_state = DETECT;

  //FSM
  switch (running_state) {
    case DETECT:
      running_state = detect();
      break;
    case FIND_FIRE:
      running_state = find_fire();
      break;
    case END:
      running_state = complete();
      break;
  };

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  disable_motors();
  return STOPPED;
}


//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_front_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_front_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off
  turret_motor.detach();

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
  pinMode(turret, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
  turret_motor.attach(turret);
}

RUN_STATE detect() { // initial detection and alignment towards fire from starting point - CURRENTLY 240 DEGREES OF COVERAGE
  int servo_val = 900;
  int servo_max = 0;
  int max_sum = 0;  
  int rotation_count = 1;
  int max_rotation_count = 1;
  int count = 0;

  while(rotation_count <= 3) {
    while(count < 10) { // full range rotation - CAN PERHAPS MODIFY FOR FULL 360 DEGREES
      
      if(rotation_count % 2 == 1) {
        servo_val += (600 / 10); // 600 is range from centre, 10 is number of increments
      }
      else {
        servo_val -= (600 / 10);
      }
      
      turret_motor.writeMicroseconds(servo_val);
      phototransistors();
  
      if(PT_sum > max_sum) {
        servo_max = servo_val;
        max_sum = PT_sum;
        max_rotation_count = rotation_count;
      }

      count++;
      delay(10);
    }
    rotate(120);
    count = 0;
    rotation_count++;
  }

  turret_motor.writeMicroseconds(1500); // reset to default
  rotate(120 * (max_rotation_count % 3) + ((servo_max - 1500) / 10)); // orients robot towards fire

  return FIND_FIRE;
}


RUN_STATE find_fire() {
  static FIRE_FIGHTING_STATE state = FORWARD_DEFAULT;
  
  //FSM
  switch(state) {
    case FORWARD_DEFAULT:
      state = forward_default();
      break;
    case FORWARD_PASS:
      state = forward_pass();
      break;
    case STRAFE_LEFT:
      state = strafe_left();
      break;
    case STRAFE_RIGHT:
      state = strafe_right();
      break;
    case EXTINGUISH:
      state = extinguish();
      break;
  };

  if(numFires == 1 && extinguished) {
    return DETECT;
  }
  else if(numFires == 2) {
    return END;
  }
}

RUN_STATE complete() {
  stop();
  disable_motors();
}

FIRE_FIGHTING_STATE forward_default() { // default driving forward
  Ultrasound();
  IR_Sensors();
  gyro();
  sweep();
  
  left_front_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);  

  if(Ultradistance < 20 || IR_MID_1_DIST < 20 || IR_MID_2_DIST < 20) {
    stop();
    phototransistors();
    isFireClose();
    
    if(fire_is_close) {
      return EXTINGUISH;
    }
    else if(IR_MID_1_DIST < IR_MID_2_DIST) {
      return STRAFE_LEFT;
    }
    else {
      return STRAFE_RIGHT;
    }
  }
  else {
    return FORWARD_DEFAULT; // keep going forward
  }
}

FIRE_FIGHTING_STATE forward_pass() { // driving forward to pass obstacle
  IR_Sensors();
  gyro();

  left_front_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + forward_speed - gyroCorrection);  
  delay(1000);
  
  if(strafed_right) {
    return STRAFE_LEFT;
  }
  else if(strafed_left) {
    return STRAFE_RIGHT;
  }
}

FIRE_FIGHTING_STATE strafe_left() {
  IR_Sensors();
  gyro();

  left_front_motor.writeMicroseconds(1500 - strafe_speed - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + strafe_speed - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 + strafe_speed - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 - strafe_speed - gyroCorrection); 

  if(IR_MID_2_DIST < 25) {
    return STRAFE_LEFT;
  }
  else if(strafed_right) {
    strafed_right = false;
    delay(500);
    return FORWARD_DEFAULT;
  }
  else {
    strafed_left = true;
    delay(250);
    stop();
    return FORWARD_PASS;
  }
}

FIRE_FIGHTING_STATE strafe_right() {
  IR_Sensors();
  gyro();

  left_front_motor.writeMicroseconds(1500 + strafe_speed - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 - strafe_speed - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - strafe_speed - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + strafe_speed - gyroCorrection); 

  if(IR_MID_1_DIST < 25) {
    return STRAFE_RIGHT;
  }
  else if(strafed_left) {
    strafed_left = false;
    delay(500);
    return FORWARD_DEFAULT;
  }
  else {
    strafed_right = true;
    delay(250);
    stop();
    return FORWARD_PASS;
  }
}

FIRE_FIGHTING_STATE extinguish() {
  numFires++;
  
  alignFan();
  digitalWrite(mosfetPin, HIGH);
  delay(10000);
  digitalWrite(mosfetPin, LOW);

  extinguished = true;
  return FORWARD_DEFAULT;
}

void rotate(float angle) { // Drives robot straight in x, y, and z // turning only occurs by itself i.e. no x and y
  DRIVING = true;

  reference[2] = angle * (PI / 180);

  while (DRIVING) {
    Controller(); // CONTROL LOOP
  }

  totalTime = 0;
  currentAngle = 0;
  accelerated = false;

  stop(); // stop motors
}

void sweep() {
  static int servo_val = 1500;
  static int max_servo_val = 0;
  static int max_PT_sum = 0;
  static bool CCW = true;
  static int ccw_val = 2100;
  static int cw_val = 900;
  phototransistors();

  if(PT_sum > max_PT_sum) {
    max_PT_sum = PT_sum;
    max_servo_val = servo_val; 
  }

  if(CCW) {
    servo_val += 100;
    if(servo_val == ccw_val) {
      CCW = false;
    }
  }
  else if(!CCW) {
    servo_val -= 100;
    if(servo_val == cw_val) {
      CCW = true;
      rotate((max_servo_val - 1500) / 10); // reorient towards fire
      servo_val = 1500;
    }
  }
  
  turret_motor.writeMicroseconds(servo_val);
}

void isFireClose() {
  if(PT_sum > detection_threshold) {
    fire_is_close = true;
  }
}

void alignFan() {
  int servo_val = 1500;
  phototransistors();

  while(abs(PT2_reading - PT3_reading) < 10) {
    if(PT2_reading > PT3_reading) { // ccw
      servo_val += 50;
      turret_motor.writeMicroseconds(servo_val);
    }
    else { // cw
      servo_val -= 50;
      turret_motor.writeMicroseconds(servo_val);
    }

    delay(10);
    phototransistors();
  }
}

//----OPEN LOOP DRIVING FUNCTIONS----
void stop() //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}

void ccw(int speedval)
{
  left_front_motor.writeMicroseconds(1500 - speedval);
  left_rear_motor.writeMicroseconds(1500 - speedval);
  right_rear_motor.writeMicroseconds(1500 - speedval);
  right_front_motor.writeMicroseconds(1500 - speedval);
}

void cw(int speedval)
{
  left_front_motor.writeMicroseconds(1500 + speedval);
  left_rear_motor.writeMicroseconds(1500 + speedval);
  right_rear_motor.writeMicroseconds(1500 + speedval);
  right_front_motor.writeMicroseconds(1500 + speedval);
}
//----OPEN LOOP DRIVING FUNCTIONS----

void Controller() {

  gyro();
  Ultrasound();
  IR_Sensors();
  //----Replacement for stuff in DriveXYZ----

  if (reference[0] == 0) {
    error[0] = 0;
  }
  else {
    error[0] = reference[0] - Ultradistance;
  }

  if (reference[1] == 0) {
    error[1] = 0;
  }
  else {
    error[1] = reference[1] - IR_wall_dist;
  }

  if (reference[2] == 0) {
    error[2] = 0;
  }
  else {
    error[2] = reference[2] - radiansAngle;
    correction = 0;
    gyroCorrection = 0;
  }

  //----Replacement for stuff in DriveXYZ----

  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();

  // exit loop
  if ((abs(error[0]) + abs(error[1])) <= 2.5 && (reference[2] == 0)) { // XY exit condition - COULD BE TWEAKED ALONGSIDE GAINS
    DRIVING = false;
    return;
  }
  else if (abs(error[2]) < 0.1 && (!(reference[2] == 0))) { // turning exit condition
    DRIVING = false;
    currentAngle = 0;
    return;
  }

  delay(10);
}

void PID_Controller() {

  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  totalTime += elapsedTime;

  for (int i = 0; i < 3; i++) {

    Pterm[i] = Kp[i] * error[i];
    Iterm[i] += Ki[i] * error[i] * elapsedTime;
    Dterm[i] = Kd[i] * ((error[i] - lastError[i]) / elapsedTime);

    // anti wind-up - Iterm and Pterm (More powerful anti windup - constrains total control effort to always be <= max velocity)
    if (abs(Pterm[i]) > max_velocity[i]) {

      // constrains Iterm such that Pterm + Iterm <= maximum velocity
      if (Pterm[i] < 0) {
        Pterm[i] = (-1 * max_velocity[i]);
      }
      else {
        Pterm[i] = max_velocity[i];
      }
    }

    if (abs(Iterm[i] + Pterm[i]) > max_velocity[i]) {

      // constrains Iterm such that Pterm + Iterm <= maximum velocity
      if ((Iterm[i] + Pterm[i]) < 0) {
        Iterm[i] = (-1 * max_velocity[i]) - Pterm[i];
      }
      else {
        Iterm[i] = max_velocity[i] - Pterm[i];
      }
    }

    velocity[i] = Pterm[i] + Iterm[i] + Dterm[i];

    // constrain to max velocity - just to ensure velocity <= maximum velocity
    if (abs(velocity[i]) > max_velocity[i]) {
      if (velocity[i] < 0) {
        velocity[i] = -1 * max_velocity[i];
      }
      else {
        velocity[i] = max_velocity[i];
      }
    }
    
    //accelerate
    if(totalTime < 0.5) {
      velocity[i] = (totalTime / 0.5) * velocity[i];
    }
    else {
      accelerated = true;
    }

    lastError[i] = error[i];
  }
  previousTime = currentTime;
}

void inverse_kinematics() {
  ang_vel[0] = (-velocity[0] + velocity[1] + ((L + l) * velocity[2])) / R_w; // left front
  ang_vel[1] = (-velocity[0] - velocity[1] + ((L + l) * velocity[2])) / R_w; // left rear
  ang_vel[2] = (velocity[0] - velocity[1] + ((L + l) * velocity[2])) / R_w; // right rear
  ang_vel[3] = (velocity[0] + velocity[1] + ((L + l) * velocity[2])) / R_w; // right front
}

void set_motor_speed() {
  for (int i = 0; i < 4; i++) {
    motor_speed_Value[i] = ang_vel[i] * 28; // scale angular velocity to motor speed value - COULD BE TUNED BETTER
  }
}

void set_motors() {
  left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0] - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1] - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2] - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3] - gyroCorrection);
}

void Ultrasound() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  Ultraduration = pulseIn(echoPin, HIGH);

  Ultradistance = ultra_centre_offset + (Ultraduration * 0.034 / 2);
}

void IR_Sensors() {
  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT);
  IR_LONG_2_DIST = IR_dist(RIGHT);
  IR_MID_1_DIST = IR_dist(FRONT_LEFT);
  IR_MID_2_DIST = IR_dist(FRONT_RIGHT);
}

void gyro() { // could be tuned better
  gyroTime = millis();

  timeElapsed = gyroTime - prev_gyroTime;

  gyroRate = ((analogRead(gyroPin) - gyroMiddle) * gyroSupplyVoltage) / 1024.0;
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second

  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    angleChange = angularVelocity * (timeElapsed / 1000.0);
    currentAngle += angleChange;
    gyroCorrection = currentAngle * Kp_gyro;
    radiansAngle = currentAngle * (PI / 180.0);
  }

  delay(5);

  prev_gyroTime = gyroTime;
}

float IR_dist(IR code) { // find distances using calibration curve equations
  float est, dist;
  int adc;

  switch (code) {
    case LEFT:
      adc = analogRead(IR_LONG_1);
      if (adc != 0 && adc <= 650) {
        dist = (13391) / (pow(adc, 1.172));
        est = Kalman(dist, last_est[0], last_var[0], LEFT);

        //MA FILTER
        SUM[0] -= FRONT_LIR[index[0]];
        FRONT_LIR[index[0]] = est;
        SUM[0] += est;
        index[0] = (index[0] + 1) % WINDOW_SIZE;
        averaged[0] = SUM[0] / WINDOW_SIZE;
        est = averaged[0];
        last_est[0] = averaged[0];
        //MA FILTER
      } else {
        est = last_est[0];
      }
      break;
    case RIGHT:
      adc = analogRead(IR_LONG_2);
      if (adc != 0 && adc <= 650) {
        dist = (11852) / (pow(adc, 1.153));
        est = Kalman(dist, last_est[1], last_var[1], RIGHT);

        //MA FILTER
        SUM[1] -= BACK_LIR[index[1]];
        BACK_LIR[index[1]] = est;
        SUM[1] += est;
        index[1] = (index[1] + 1) % WINDOW_SIZE;
        averaged[1] = SUM[1] / WINDOW_SIZE;
        est = averaged[1];
        last_est[1] = averaged[1];
        //MA FILTER
      } else {
        est = last_est[1];
      }
      break;
    case FRONT_LEFT:
      adc = analogRead(IR_MID_1);
      if (adc != 0 && adc <= 650) {
        dist = (3730.6) / (pow(adc, 1.082));
        est = Kalman(dist, last_est[2], last_var[2], FRONT_LEFT);

        //MA FILTER
        SUM[2] -= LEFT_MIR[index[2]];
        LEFT_MIR[index[2]] = est;
        SUM[2] += est;
        index[2] = (index[2] + 1) % WINDOW_SIZE;
        averaged[2] = SUM[2] / WINDOW_SIZE;
        est = averaged[2];
        last_est[2] = averaged[2];
        //MA FILTER
      } else {
        est = last_est[2];
      }
      break;
    case FRONT_RIGHT:
      adc = analogRead(IR_MID_2);
      if (adc != 0 && adc <= 650) {
        dist = (3491.3) / (pow(adc, 1.069));
        est = Kalman(dist, last_est[3], last_var[3], FRONT_RIGHT);

        //MA FILTER
        SUM[3] -= RIGHT_MIR[index[3]];
        RIGHT_MIR[index[3]] = est;
        SUM[3] += est;
        index[3] = (index[3] + 1) % WINDOW_SIZE;
        averaged[3] = SUM[3] / WINDOW_SIZE;
        est = averaged[3];
        last_est[3] = averaged[3];
        //MA FILTER
      } else {
        est = last_est[3];
      }
      break;
  }

  delay(1);

  return est;
}

// Kalman Filter for IR sensors
float Kalman(float rawdata, float prev_est, float last_variance, IR code) {
  float a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_variance + process_noise;

  kalman_gain = a_priori_var / (a_priori_var + sensor_noise);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;

  switch (code) {
    case LEFT:
      last_var[0] = a_post_var;
    case RIGHT:
      last_var[1] = a_post_var;
    case FRONT_LEFT:
      last_var[2] = a_post_var;
    case FRONT_RIGHT:
      last_var[3] = a_post_var;
  }

  return a_post_est;
}

void phototransistors() {
  PT1_reading = analogRead(PT1_pin); // left-most
  PT2_reading = analogRead(PT2_pin); // left-middle
  PT3_reading = analogRead(PT3_pin); // right-middle
  PT4_reading = analogRead(PT4_pin); // right-most

  PT_sum = PT1_reading + PT2_reading + PT3_reading + PT4_reading;
}
//----WRITTEN HELPER FUNCTIONS----
