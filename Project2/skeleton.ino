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
  FIND_FIRE,
  EXTINGUISH,
  END,
};

//----RUNNING VARIABLES----
int switch_back_count = 0;
bool forward_flag = false;
bool turned = false; // after turn in middle -> true
//----RUNNING VARIABLES----

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

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
  LEFT_FRONT,
  LEFT_BACK,
  BACK_LEFT,
  BACK_RIGHT,
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

const int PT1_pin = 0;
const int PT2_pin = 0;
const int PT3_pin = 0;
const int PT4_pin = 0;

int PT1_reading = 0;
int PT2_reading = 0;
int PT3_reading = 0;
int PT4_reading = 0;
//----Phototransistor----

//----Fan-----

//----Fan-----

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
float Kp_gyro = 30;

float radiansAngle = 0;
//----Gyro----

//----Driving----
bool DRIVING = false;
bool corner_finished = false;
bool is_turning = false;
bool is_driving_middle = false;
bool is_strafing = false;
bool accelerated = false;
//----Driving----

//----Fire Fighting----
int numFires = 0;
//----Fire Fighting----

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29

//----InverseKinematicValues----
float L = 007.620;
float l = 009.078;
float R_w = 002.54;

float velocity[3] = {0, 0, 0};
float max_velocity[3] = {30, 20, 1.5}; // cm/s
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


// PID VALUES WHEN WE WANT HIGHER CONTROL EFFORT FROM THE Y CONTROLLER RELATIVE TO THE X CONTROLLER
//float Kp_original[3] = {2,1.8,1.65};
//float Kp_amplified[3] = {2,3,1.65};

// PID VALUES FOR X AND Y NEED TO BE TUNED AND TESTED - TURNING SHOULD BE FINE
float Kp[3] = {2.5, 2.2, 1.65};
float Ki[3] = {0.1, 0.03, 0.05};
float Kd[3] = {0, 0, 0};

float Kp_straight = 45; // SHOULD BE TUNED
float Kp_turn = 1500;
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

  static RUN_STATE running_state = FIND_FIRE;

  //FSM
  switch (running_state) {
    case FIND_FIRE:
      running_state = find_fire();
      break;
    case EXTINGUISH:
      running_state = put_out_fire();
      break;
    case END:
      running_state = complete();
      break;
      //----DRIVING COMPONENT----
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

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_front_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_front_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}

RUN_STATE find_fire() {
  sweep();
  if(isFireDetected()) {
    
  }

  return EXTINGUISH;
}

RUN_STATE put_out_fire() {
  numFires++;

  if(numFires == 2) {
    return END;
  }
  else {
    return FIND_FIRE;
  }
}

RUN_STATE complete() {
  stop();
  disable_motors();
}

void rotate_small(float angle) { // OLD ROTATE FUNCTION - DOESNT USE KINEMATICS
  float power;

  angle = angle * (PI / 180);

  currentAngle = 0;

  do {
    gyro();
    power = (angle - radiansAngle) * Kp_turn;

    // constrain
    if (power > 130) {
      power = 130;
    }
    else if (power < -130) {
      power = -130;
    }

    // send power
    left_front_motor.writeMicroseconds(1500 + power);
    left_rear_motor.writeMicroseconds(1500 + power);
    right_rear_motor.writeMicroseconds(1500 + power);
    right_front_motor.writeMicroseconds(1500 + power);
  } while (abs(radiansAngle) < abs(angle)); // exit condition

  return;
}

void driveXYZ(float x, float y, float z) { // Drives robot straight in x, y, and z // turning only occurs by itself i.e. no x and y
  DRIVING = true;

  reference[0] = x;
  reference[1] = y;
  reference[2] = z * (PI / 180);

  if(y >= 180) { // reveresing gains
    Kp[0] = 0.48;
    Ki[0] = 0.001;
  }

  if(is_strafing) {
    Kp[1] = 2.2;
    Ki[1] = 0.015;
    Kd[1] = 0.1;
  }

  while (DRIVING) {
    Controller(); // CONTROL LOOP
  }

  // reset gains
  Kp[0] = 2.7;
  Ki[0] = 0.1;
  Kp[1] = 2.2;
  Ki[1] = 0.03;
  Kd[1] = 0;

  totalTime = 0;
  accelerated = false;

  is_strafing = false;
  is_turning = false;

  stop(); // stop motors
}

void sweep() {
  
}

bool isFireDetected() {
  
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

  if(correction > 100) {
    correction = 100;
  }
  else if(correction < -100) {
    correction = -100;
  }

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
    is_turning = true;
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

    //    // OTHER ANTI-WINDUP - TUNE PID VALUES
    //    if(abs(Iterm[i]) > max_velocity[i]) {
    //
    //      // constrains Iterm such that Pterm + Iterm <= maximum velocity
    //      if(Iterm[i] < 0) {
    //        Iterm[i] = (-1 * max_velocity[i]);
    //      }
    //      else {
    //        Iterm[i] = max_velocity[i];
    //      }
    //    }

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

  if (switch_back_count == 0 || switch_back_count == 8) { // only use at the edges
    // IR straight
    left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0] - correction);
    left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1] - correction);
    right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2] - correction);
    right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3] - correction);
  }
  else {
    // gyro straight
    left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0] - gyroCorrection);
    left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1] - gyroCorrection);
    right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2] - gyroCorrection);
    right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3] - gyroCorrection);
  }
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
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);

  IR_MID_1_DIST = IR_dist(BACK_LEFT);
  IR_MID_2_DIST = IR_dist(BACK_RIGHT);

  IR_wall_dist = long_centre_offset + (0.37 * IR_LONG_1_DIST + 0.63 * IR_LONG_2_DIST); // distance of centre of robot to wall
  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs

  IR_mid_dist = mid_centre_offset + ((IR_MID_1_DIST + IR_MID_2_DIST) / 2);
  IR_mid_diff = IR_MID_1_DIST - IR_MID_2_DIST;

  if (abs(IR_diff) > 0.2 && (accelerated)) {
    correction = Kp_straight * IR_diff;
  }
  else {
    correction = 0;
  }
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
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      if (adc != 0 && adc <= 650) {
        dist = (13391) / (pow(adc, 1.172));
        est = Kalman(dist, last_est[0], last_var[0], LEFT_FRONT);

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
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      if (adc != 0 && adc <= 650) {
        dist = (11852) / (pow(adc, 1.153));
        est = Kalman(dist, last_est[1], last_var[1], LEFT_BACK);

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
    case BACK_LEFT:
      adc = analogRead(IR_MID_1);
      if (adc != 0 && adc <= 650) {
        dist = (3730.6) / (pow(adc, 1.082));
        est = Kalman(dist, last_est[2], last_var[2], BACK_LEFT);

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
    case BACK_RIGHT:
      adc = analogRead(IR_MID_2);
      if (adc != 0 && adc <= 650) {
        dist = (3491.3) / (pow(adc, 1.069));
        est = Kalman(dist, last_est[3], last_var[3], BACK_RIGHT);

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
    case LEFT_FRONT:
      last_var[0] = a_post_var;
    case LEFT_BACK:
      last_var[1] = a_post_var;
    case BACK_LEFT:
      last_var[2] = a_post_var;
    case BACK_RIGHT:
      last_var[3] = a_post_var;
  }

  return a_post_est;
}

void phototransistors() {
  PT1_reading = analogRead(A9);
  PT2_reading = analogRead(A9);
  PT3_reading = analogRead(A9);
  PT4_reading = analogRead(A9);
}


//----WRITTEN HELPER FUNCTIONS----
