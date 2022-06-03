#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>
#include <SharpDistSensor.h>

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
  REALIGN,
  STRAFE_LEFT,
  STRAFE_RIGHT,
  EXTINGUISH,
  REPOSITION,
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;
const byte turret = 25;

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
const float ultra_centre_offset = 3.5; // from ultrasonic to fan
//----Ultrasound----

//----IR----
enum IR {
  FRONT_LEFT, // long
  FRONT_RIGHT, // long
  LEFT, // mid
  RIGHT, // mid
};

// Left long range IR
const int IR_LONG_1 = A6;
float IR_LONG_1_DIST = 0;

// Right long range IR
const int IR_LONG_2 = A7;
float IR_LONG_2_DIST = 0;

// Front left mid range IR
const int IR_MID_1 = A4;
float IR_MID_1_DIST = 0;

// Front right mid range IR
const int IR_MID_2 = A5;
float IR_MID_2_DIST = 0;

const float IR_LONG_OFFSET = 4.0; // distance from front IR to fan
//----IR----

//----IR Kalman Filter----
float last_est[4] = {0, 0, 0, 0};
float last_var[4] = {999, 999, 999, 999};
float process_noise = 5;
float sensor_noise = 6;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----Boolean variables for which sensor detects obstacle----
bool left_IR_close = false; 
bool right_IR_close = false;
bool ultrasonic_close = false;
bool l_side_IR_close = false;
bool r_side_IR_close = false;
//----Boolean variables for which sensor detects obstacle----

//----Phototransistors----
const int PT1_pin = A9;
const int PT2_pin = A10;
const int PT3_pin = A12;
const int PT4_pin = A13;

int PT1_reading = 0;
int PT2_reading = 0;
int PT3_reading = 0;
int PT4_reading = 0;
float PT_left = 0;
float PT_right = 0;

int PT_diff = 0;
float PT_ratio = 0;
float PT_sum = 0;

float Kp_align = 150;
float PT_correction = 0;

int repos_time = 0; // time for robot to drive forward to reposition
int min_detect_threshold = 60; // minimum value to know if fire is present - SUBJECT TO CHANGE 
const float detection_threshold = 1700; // when close to fire - SUBJECT TO CHANGE
//----Phototransistors----

//----Obstacle Avoidance----
int forward_time = 0;
int obstacle_passed_time = 0;
int obstacle_passed_time_start = 0;
bool initial_left = true;
bool initial_right = true;
bool obstacle_left = false;
bool obstacle_right = false;
bool only_once_left = false;
bool only_once_right = false;
float obstacle_left_min = 99;
float obstacle_right_min = 99;
float minimum_left = 0;
float minimum_right = 0;

bool strafed_left = false;
bool strafed_right = false;
bool strafe_reversed = false;
bool strafe_OL = false;
bool left_detected = false;
bool right_detected = false;
bool between = false;
bool repositioned = false;
int strafe_left_time = 0;
int strafe_right_time = 0;
int strafe_reverse_count = 0;

bool pass = false;
bool pass_IR_detected = false;
bool pass_OL = false;
int pass_time = 0;

int extra_powerL = 0;
int extra_powerR = 0;
int extra_power_passL = 0;
int extra_power_passR = 0;
//----Obstacle Avoidance----

//----Fire Fighting----
const int mosfetPin = 45; // mosfet pin for fan

//########
//int servo_val = 0;
int align_servo_val = 1445; // facing forward
//########

int numFires = 0;
bool at_wall = false;
bool fire_is_close = false;
bool fanAligned = false;
bool extinguished = false;
//----Fire Fighting----

//----MA Filter----
#define WINDOW_SIZE 13
int index[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float value[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float SUM[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO

float LEFT_LIR[WINDOW_SIZE];
float RIGHT_LIR[WINDOW_SIZE];
float LEFT_MIR[WINDOW_SIZE];
float RIGHT_MIR[WINDOW_SIZE];
float SONAR[6];

// Filters for PT - PROBABLY DONT NEED
float LEFT_PT_1[WINDOW_SIZE];
float LEFT_PT_2[WINDOW_SIZE];
float RIGHT_PT_1[WINDOW_SIZE];
float RIGHT_PT_2[WINDOW_SIZE];

float averaged[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO, [6]-[9]PT
//----MA Filter----

//Median Filtering using the sharp library -------------
const byte medianFilterWindowSize = 5;
const byte medianFilterWindowSize1 = 11;

SharpDistSensor LEFT_LONG(IR_LONG_1, medianFilterWindowSize);
SharpDistSensor RIGHT_LONG(IR_LONG_2, medianFilterWindowSize);
SharpDistSensor LEFT_MID(IR_MID_1, medianFilterWindowSize1);
SharpDistSensor RIGHT_MID(IR_MID_2, medianFilterWindowSize1);

const float C_L1 = 7058.6;
const float P_L1 = -1.066;
const float C_L2 = 11178;
const float P_L2 = -1.14;
const float C_M1 = 5586.2;
const float P_M1 = -1.15;
const float C_M2 = 452.63;
const float P_M2 = -0.728;

const unsigned int minVal_L1 = 40; // ~800 mm
const unsigned int maxVal_L1 = 500; // ~50mm
const unsigned int minVal_L2 = 130; // ~800 mm
const unsigned int maxVal_L2 = 500; // ~50mm
const unsigned int minVal_M1 = 90; // ~800 mm
const unsigned int maxVal_M1 = 410; // ~50mm
const unsigned int minVal_M2 = 35; // ~800 mm
const unsigned int maxVal_M2 = 400; // ~50mm

//Median Filtering using the sharp library -------------

//----Gyro----
const int gyroPin = A8;
int gyroADC = 0;
float gyroSupplyVoltage = 5; 
const float gyroMiddle = 505;
float gyroSensitivity = 0.007; 
float rotationThreshold = 1;      
float angularVelocity = 0;

int timeElapsed = 0;
int prev_gyroTime = 0;
int gyroTime = 0;
float gyroRate = 0;                                   

float angleChange = 0;
float currentAngle = 0;      
float radiansAngle = 0;
float Kp_gyro = 30;
float gyroCorrection = 0;
//----Gyro----

//----Driving----
bool TURNING = false;
bool accelerated = false;
bool first_call = true;
//----Driving----

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//----InverseKinematicValues---- ONLY FOR TURNING
float L = 007.620;
float l = 009.078;
float R_w = 002.54;

float velocity = 0;
float max_velocity = 1.6; // rad/s
float ang_vel[4] = {0, 0, 0, 0};

float motor_speed_Value[4] = {0, 0, 0, 0};
//----InverseKinematicValues----

float reference = 0;
float currentTime, previousTime, elapsedTime;
float turningTime = 0; // for acceleration when turning
//----PIDValues----
float drivingTime = 0; // for acceleration when driving X and Y
float initalTime = 0;
float error = 0;
float lastError = 0;
float rateError = 0;

float Pterm, Iterm, Dterm;

// PID VALUES FOR ROTATION
float Kp = 1.8;
float Ki = 0.1;
float Kd = 0;

float Kp_turn = 1250; // for rotate_small function
//----PIDValues----

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  LEFT_LONG.setPowerFitCoeffs(C_L1, P_L1, minVal_L1, maxVal_L1);
  RIGHT_LONG.setPowerFitCoeffs(C_L2, P_L2, minVal_L2, maxVal_L2);
  LEFT_MID.setPowerFitCoeffs(C_M1, P_M1, minVal_M1, maxVal_M1);
  RIGHT_MID.setPowerFitCoeffs(C_M2, P_M2, minVal_M2, maxVal_M2);
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

RUN_STATE detect() { // initial detection and alignment towards fire from starting point
  
  int power = 200;
  int count = 0;
  float ratio_threshold = 1.15;
  bool CCW = false;                                                                                                                                                                                                 

  turret_motor.writeMicroseconds(1445);

  while(count < 100) {
    phototransistors();
    PT_left += PT_left;
    PT_right += PT_right;
    count++;
    delay(5);
  }  

  PT_left = PT_left / 100;
  PT_right = PT_right / 100;

  if(PT_left > PT_right) {
    CCW = true;
    ccw(power);
  }
  else {
    CCW = false;
    cw(power);
  }

  while(1) {
    phototransistors();

    if(PT_ratio < 1) { // right > left
      PT_ratio = 1.0 / PT_ratio;
    }
    
    if(PT_sum > min_detect_threshold && PT_ratio < ratio_threshold) {
      stop();
      break;
    }
    else {
      if(PT_ratio < 1.8) { // slow motors as approaches
        power = 200 * (PT_ratio / 2);
      }
      else {
        power = 200;
      }

      if(CCW) {
        ccw(power);
      }
      else {
        cw(power);
      }
    }

    delay(5);
  }

  return FIND_FIRE;
}


RUN_STATE find_fire() {
  static FIRE_FIGHTING_STATE state = FORWARD_DEFAULT;
  
  //FSM for fire fighting
  switch(state) {
    case FORWARD_DEFAULT:
      state = forward_default();
      break;
    case FORWARD_PASS:
      state = forward_pass();
      break;
    case REALIGN:
      state = realign();
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
    case REPOSITION: // if no fire detected in DETECT, reposition robot
      state = reposition();
      break;
  };

  if(repositioned) {
    repositioned = false;
    return DETECT;
  }

  if(numFires == 1 && extinguished) { // if one fire has just been extinguished - find other fire
    extinguished = false;
    min_detect_threshold = 40; // lower detection threshold for other fire
    forward_straight(-150);
    delay(200);
    stop();
    state = FORWARD_DEFAULT;
    //reset();
    return DETECT;
  }
  else if(numFires == 2) { // end when all fires detected
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

FIRE_FIGHTING_STATE forward_default() { // default driving forward
  int power = 190; // Could potentially decrease power depending on how close to obstacle - making stopping less abrupt
  int left_dist = 0;
  static int right_dist = 0;
  pass = false;
  strafe_reverse_count = 0;

  Ultrasound();
  IR_Sensors();
  Update(); // alignment to fire using PTs and applying gain

  if(IR_MID_1_DIST <= 24) {
    left_dist = IR_MID_1_DIST;
    if(left_dist < obstacle_left_min) {
      obstacle_left_min = left_dist;
    }
    initial_left = true;
    obstacle_left = true;
    only_once_left = true;
  }
  else {
    if(only_once_left) {
      minimum_left = obstacle_left_min;
      Serial.println(minimum_left);
      obstacle_left_min = 999;
      only_once_left = false;
    }
    if(obstacle_left) {
      if(initial_left) {
        obstacle_passed_time_start = millis();
        initial_left = false;
      }
      obstacle_passed_time = millis();
    }
  }

  if(IR_MID_2_DIST <= 24) {
    right_dist = IR_MID_2_DIST;
    if(right_dist < obstacle_right_min) {
      obstacle_right_min = right_dist;
    }
    initial_right = true;
    obstacle_right = true;
    only_once_right = true;
  }
  else {
    if(only_once_right) {
      minimum_right = obstacle_right_min;
      obstacle_right_min = 999;
      only_once_right = false;
    }
    if(obstacle_right) {
      if(initial_right) {
        obstacle_passed_time_start = millis();
        initial_right = false;
      }
      obstacle_passed_time = millis();
    }
  }
  
  // SIDE OBSTACLE DETECTION
  if(IR_MID_1_DIST <= 9) {
    l_side_IR_close = true;
  }
  else {
    l_side_IR_close = false;
  }
  
  if(IR_MID_2_DIST <= 9) {
    r_side_IR_close = true;
  }
  else {
    r_side_IR_close = false;
  }

  if(l_side_IR_close && r_side_IR_close) {
    extra_powerL = 0;
    extra_powerR = 0;
  }
  else if(l_side_IR_close) {
    extra_powerR = 90;
    extra_powerL = 0;
    PT_correction = 0;
  }
  else if(r_side_IR_close) {
    extra_powerL = 90;
    extra_powerR = 0;
    PT_correction = 0;
  }
  else {
    extra_powerL = 0;
    extra_powerR = 0;
  }

  if(Ultradistance < 20) {
    power = 150;
  }

  forward_towards_fire(power); // set motor power forward

  // FRONT OBSTACLE DETECTION
  if(Ultradistance < 14 || IR_LONG_1_DIST <= 16 || IR_LONG_2_DIST <= 16) {

    stop();
    delay(450);

    currentAngle = 0;
    radiansAngle = 0;

    l_side_IR_close = false;
    r_side_IR_close = false;
    
    if(Ultradistance < 15) {
      ultrasonic_close = true;
    }
    else if(IR_LONG_1_DIST <= 16) {
      left_IR_close = true;
    }
    else {
      right_IR_close = true;
    }
    
    //is_fire_close();
    //Serial.println(minimum_left);

    phototransistors();

    if(PT_left > detection_threshold / 2 || PT_right > detection_threshold / 2) {
      return EXTINGUISH;
    }
    else if((obstacle_passed_time - obstacle_passed_time_start) < 1000 && (obstacle_passed_time - obstacle_passed_time_start) > 0) {
      ultrasonic_close = false;
      first_call = true;
      
      if(obstacle_left) {
        obstacle_left = false;
        obstacle_right = false;
        if(right_IR_close) {
          right_IR_close = false;
          left_IR_close = false;
          if(minimum_left <= 13) {
            return STRAFE_RIGHT;
          }
          else {
            return STRAFE_LEFT;
          }
        }
        else {
          left_IR_close = false;
          right_IR_close = false;
          return STRAFE_RIGHT;
        }
      }
      else if(obstacle_right) {
        obstacle_right = false;
        obstacle_left = false;
        if(left_IR_close) {
          right_IR_close = false;
          left_IR_close = false;
          if(minimum_right <= 13) {
            return STRAFE_LEFT;
          }
          else {
            return STRAFE_RIGHT;
          }
        }
        else {
          left_IR_close = false;
          right_IR_close = false;
          return STRAFE_LEFT;
        }
      }
    }   
    else if(left_IR_close) { // direction of strafe depending on front IR readings
      left_IR_close = false;
      first_call = true;
      obstacle_right = false;
      obstacle_left = false;
      if(IR_MID_2_DIST <= 20) { // if not enough space on the right
        return STRAFE_LEFT;
      }
      else {
        return STRAFE_RIGHT;
      }
    }
    else if(right_IR_close) {
      right_IR_close = false;
      first_call = true;
      obstacle_right = false;
      obstacle_left = false;
      if(IR_MID_1_DIST <= 20) { // if not enough space on the left
        return STRAFE_RIGHT;
      }
      else {
        return STRAFE_LEFT;
      }
    }
    else { // ultrasonic used to stop
      ultrasonic_close = false;
      first_call = true;
      obstacle_right = false;
      obstacle_left = false;
      if(IR_MID_1_DIST > IR_MID_2_DIST) { // strafe left if more space on left side
        return STRAFE_LEFT;
      }
      else {
        return STRAFE_RIGHT;
      }
    }
  }
  else {
    return FORWARD_DEFAULT; // keep going forward
  }
}

FIRE_FIGHTING_STATE forward_pass() { // driving forward to pass obstacle
  int power = 180;
  pass = true;

  strafe_reverse_count = 0;

  forward_straight(power); // drive forward

  IR_Sensors();
  Ultrasound();
  if(extra_power_passL == 0 && extra_power_passR == 0) {  // if no obstacle
    Gyro();
  }
  else {
    // reset gyro variables
    currentAngle = 0;
    radiansAngle = 0;
    gyroCorrection = 0;
  }

  if(strafed_left) {
    if(IR_MID_1_DIST <= 7) {
      extra_power_passR = 75;
      extra_power_passL = 0;
      first_call = true;
    }
    else {
      extra_power_passR = 0;
      extra_power_passL = 0;
    }

    if(IR_MID_2_DIST <= 15) {
      pass_IR_detected = true;
    }

    if(!pass_OL) {
      if(pass_IR_detected && IR_MID_2_DIST >= 25) {
        pass_time = millis();
        pass_OL = true;
      }
    }
    else {
      if(millis() - pass_time < 250) {
        first_call = true;
        return FORWARD_PASS;
      }
      else {
        stop();
        delay(100);
        pass_IR_detected = false;
        pass_OL = false;
        return REALIGN;
      }    
    }
  }
  else if(strafed_right) {
    if(IR_MID_2_DIST <= 7) {
      extra_power_passR = 0;
      extra_power_passL = 75;
      first_call = true;
    }
    else {
      extra_power_passR = 0;
      extra_power_passL = 0;
    }

    if(IR_MID_1_DIST <= 15) {
      pass_IR_detected = true;
    }

    if(!pass_OL) {
      if(pass_IR_detected && IR_MID_1_DIST >= 25) {
        pass_time = millis();
        pass_OL = true;
      }
    }
    else {
      if(millis() - pass_time < 100) {
        first_call = true;
        return FORWARD_PASS;
      }
      else {
        stop();
        delay(100);
        pass_IR_detected = false;
        pass_OL = false;
        return REALIGN;
      }    
    }
  }

  if(Ultradistance < 14 || IR_LONG_1_DIST <= 14 || IR_LONG_2_DIST <= 14) {
    stop();
    delay(450);

    phototransistors();
    
    if(PT_left > detection_threshold / 2.5 || PT_right > detection_threshold / 2.5) {
      return EXTINGUISH;
    }
    
    pass_IR_detected = false;
    pass_OL = false;
    first_call = true;
    
    if(strafed_left) {
      return STRAFE_LEFT;
    }
    else if(strafed_right) {
      return STRAFE_RIGHT;
    }
  }

  return FORWARD_PASS;
}

FIRE_FIGHTING_STATE realign() {
  int power = 150;
  turret_motor.writeMicroseconds(1445);
  
  if(strafed_left) {                                                                                                                                                                                                
    cw(power);

    while(1) {
      phototransistors();
  
      if(PT_ratio < 1) { // right > left
        PT_ratio = 1.0 / PT_ratio;
      }
      
      if(PT_sum > 60 && PT_ratio < 1.15) {
        stop();
        strafed_left = false;
        strafed_right = false;
        initial_left = true;
        initial_right = true;
        return FORWARD_DEFAULT;
      }
      else {
        if(PT_ratio < 1.8) { // slow motors as approaches
          power = 150 * (PT_ratio / 2);
        }
        else {
          power = 150;
        }
        
        cw(power);
      }
  
      delay(5);
    }
  }
  else if(strafed_right) {
    ccw(power);

    while(1) {
      phototransistors();
  
      if(PT_ratio < 1) { // right > left
        PT_ratio = 1.0 / PT_ratio;
      }
      
      if(PT_sum > 60 && PT_ratio < 1.15) {
        stop();
        strafed_left = false;
        strafed_right = false;
        initial_left = true;
        initial_right = true;
        return FORWARD_DEFAULT;
      }
      else {
        if(PT_ratio < 1.8) { // slow motors as approaches
          power = 150 * (PT_ratio / 2);
        }
        else {
          power = 150;
        }
        
        ccw(power);
      }
  
      delay(5);
    }
  }
}

FIRE_FIGHTING_STATE strafe_left() {
  int power = -180;
  IR_Sensors();
  Gyro();

  strafe(power); // left strafe

  if(IR_MID_1_DIST <= 8) { // if left side IR detects obstacle strafe other way to avoid obstacle
      stop();
      strafed_left = false;
      right_detected = false;
      strafe_OL = false;
      strafe_reverse_count++;
      return STRAFE_RIGHT;
  }

  if(IR_LONG_2_DIST <= 19) {
    right_detected = true;
  }

  if(!strafe_OL) {
    if(right_detected && IR_LONG_2_DIST >= 27) {
      strafe_left_time = millis();
      strafe_OL = true;
    }
  }
  else {
    if(millis() - strafe_left_time < 120) {
      return STRAFE_LEFT;
    }
    else {
      stop();
      delay(150);
      right_detected = false;
      strafe_OL = false;
      strafed_left = true;
      strafed_right = false;
      first_call = true;
      return FORWARD_PASS;
    }    
  }

  return STRAFE_LEFT;
}

FIRE_FIGHTING_STATE strafe_right() { 
  int power = 180;
  IR_Sensors();
  Gyro();

  strafe(power); // right strafe

  if(IR_MID_2_DIST <= 8) { // if left side IR detects obstacle strafe other way to avoid obstacle
      stop();
      strafed_right = false;
      left_detected = false;
      strafe_OL = false;
      strafe_reverse_count++;
      return STRAFE_LEFT;
  }

  if(strafe_reverse_count > 1) {
    stop();
    strafe_reverse_count = 0;
    repos_time = millis();
    first_call = true;
    return REPOSITION;
  }
  
  if(IR_LONG_1_DIST <= 19) {
    left_detected = true;
  }

  if(!strafe_OL) {
    if(left_detected && IR_LONG_1_DIST >= 27) {
      strafe_right_time = millis();
      strafe_OL = true;
    }
  }
  else {
    if(millis() - strafe_right_time < 120) {
      return STRAFE_RIGHT;
    }
    else {
      stop();
      delay(150);
      left_detected = false;
      strafe_OL = false;
      strafed_left = false;
      strafed_right = true;
      first_call = true;
      return FORWARD_PASS;
    }    
  }

  return STRAFE_RIGHT;
}

FIRE_FIGHTING_STATE extinguish() { // extinguish fire state
  numFires++; // increment number of fires extinguished
  
  align_fan();
  put_out_fire();

  // update boolean values
  extinguished = true;
  fire_is_close = false;
  strafed_right = false;
  strafed_left = false;
  initial_left = true;
  initial_right = true;
  return FORWARD_DEFAULT;
}

FIRE_FIGHTING_STATE reposition() { // when stuck and cant move - reposition into new position and detect fire again 
  
  rotate(180);

  while(millis() - repos_time < 2000) {
    IR_Sensors();
    Ultrasound();
    Gyro();
  
    forward_straight(200);

    if(Ultradistance < 10 || IR_LONG_1_DIST <= 13 || IR_LONG_2_DIST <= 13) { // if obstacle detected
      stop();
      break;
    }
  }

  stop();

  if(IR_MID_1_DIST < IR_MID_2_DIST) {
    rotate(90);
  }
  else {
    rotate(-90);
  }

  repos_time = millis();

  while(millis() - repos_time < 2500) {
    IR_Sensors();
    Ultrasound();
    Gyro();
  
    forward_straight(200);

    if(Ultradistance < 10 || IR_LONG_1_DIST <= 13 || IR_LONG_2_DIST <= 13) { // if obstacle detected
      stop();
      break;
    }
  }

  stop();

  repositioned = true;
  initial_left = true;
  initial_right = true;
  return FORWARD_DEFAULT;
}

void rotate(float angle) { // Rotates robot using PID control - MIGHT NEED TO ADJUST GAINS DEPENDING ON ANGLE - TEST IF CCW IS +VE or -VE
  TURNING = true;
  
  if(angle < 50) { // For small angles (i.e. when realigning to face fire after sweeping) change gains - NEEDS TO BE TUNED (if doesn't work well use rotate_small function)
    Kp = 1.8 / (angle / 50);
    Ki = 0.1 * (angle / 50);
  }
  
  reference = angle * (PI / 180); // convert to radians
  
  currentAngle = 0;
  radiansAngle = 0;

  while (TURNING) {
    TurnController(); // CONTROL LOOP
  }
  
  Kp = 1.8;
  Ki = 0.1;
  turningTime = 0;
  currentAngle = 0;
  radiansAngle = 0;
  accelerated = false;

  stop(); // stop motors
}


void Update() { // FIRE ALIGNMENT IMPLEMENTATION 1

  phototransistors();

  if(PT_ratio >= 0 && PT_ratio <= 20) {
    if(PT_ratio < 1) { // right > left
      PT_ratio = 1.0 / PT_ratio;
      PT_correction = Kp_align * (PT_ratio - 1);
    }
    else {
      PT_correction = -Kp_align * (PT_ratio - 1);
    }
  }
  else {
    PT_correction = 0;
  }

  if(PT_correction > 100) { // constrain to within 100
    PT_correction = 100;
  }
  else if(PT_correction < -100) {
    PT_correction = -100;
  }
}


void is_fire_close() {
  //int prev_pos = turret_motor.read(); // read where servo is currently
  int servo_val = 1200;
  turret_motor.writeMicroseconds(servo_val); // reposition

  while(servo_val < 1800) { // sweep servo to see if fire is close
    phototransistors();
    if(PT_sum > detection_threshold) { // if greater than threshold - i.e. in 20cm range - return
      fire_is_close = true;
      turret_motor.writeMicroseconds(1445);
      return;
    }

    servo_val += 100;
    turret_motor.writeMicroseconds(servo_val);
    delay(200);
  }
  
  turret_motor.writeMicroseconds(1445); // set turret position to what is was before entering function
  return;
}

void align_fan() {
  turret_motor.writeMicroseconds(1445);
  
  phototransistors();
  
  while(abs(PT2_reading - (PT3_reading * 0.95)) > 40) { // read middle phototransistors
    if((PT2_reading) > (PT3_reading * 0.95)) { // ccw
      align_servo_val += 15; // increment ccw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
    else { // cw
      align_servo_val -= 15; // increment cw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
    
    delay(50);//****** was 10 before
    phototransistors(); // read phototransistors
  }
}

void put_out_fire() {
  // Set mosfet pin high until fire extinguished
  phototransistors(); 

  while((PT2_reading >= 250)&&(PT3_reading >= 250)){
    digitalWrite(mosfetPin, HIGH);
    delay(50);
    phototransistors();
  }
  
  digitalWrite(mosfetPin, LOW);
}

//----OPEN LOOP TURNING FUNCTIONS----
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

void forward_towards_fire(int speedval) // drives forward towards fire
{
  // remove gyroCorrection and add PT_correction if using closed loop alignement
  left_front_motor.writeMicroseconds(1500 + speedval + PT_correction - extra_powerL);
  left_rear_motor.writeMicroseconds(1500 + speedval + PT_correction - extra_powerL);
  right_rear_motor.writeMicroseconds(1500 - speedval + PT_correction + extra_powerR);
  right_front_motor.writeMicroseconds(1500 - speedval + PT_correction + extra_powerR);
}

void forward_straight(int speedval) // drives forward straight
{
  // remove gyroCorrection and add PT_correction if using closed loop alignement
  left_front_motor.writeMicroseconds(1500 + speedval - gyroCorrection - extra_power_passL);
  left_rear_motor.writeMicroseconds(1500 + speedval - gyroCorrection - extra_power_passL);
  right_rear_motor.writeMicroseconds(1500 - speedval - gyroCorrection + extra_power_passR);
  right_front_motor.writeMicroseconds(1500 - speedval - gyroCorrection + extra_power_passR);
}

void strafe(float speedval) // +ve = right, -ve = left correcting for angular rotation and x direction
{
  gyroCorrection = gyroCorrection / 2.5;
  left_front_motor.writeMicroseconds(1500 + speedval - (speedval/4.0) - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 - speedval - (speedval/4.0) - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - speedval + (speedval/4.0) - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + speedval + (speedval/4.0) - gyroCorrection);
}
//----OPEN LOOP TURNING FUNCTIONS----

void TurnController() { // controller for turning

  Gyro();
  error = reference - radiansAngle;
  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();

  // exit condition
  if (abs(error < 0.15)) { // turning exit condition
    TURNING = false;
    return;
  }

  delay(10);
}

void PID_Controller() {

  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  turningTime += elapsedTime;

  // get P, I, and D terms for velocity
  Pterm = Kp * error;
  Iterm += Ki * error * elapsedTime;
  Dterm = Kd * ((error - lastError) / elapsedTime);

  // anti wind-up - Iterm and Pterm (More powerful anti windup - constrains total control effort to always be <= max velocity)
  if (abs(Pterm) > max_velocity) {

    // constrains Iterm such that Pterm + Iterm <= maximum velocity
    if (Pterm < 0) {
      Pterm = (-1 * max_velocity);
    }
    else {
      Pterm = max_velocity;
    }
  }

  if (abs(Iterm + Pterm) > max_velocity) {

    // constrains Iterm such that Pterm + Iterm <= maximum velocity
    if ((Iterm + Pterm) < 0) {
      Iterm = (-1 * max_velocity) - Pterm;
    }
    else {
      Iterm = max_velocity - Pterm;
    }
  }

  velocity = Pterm + Iterm + Dterm; // get turning velocity 

  // constrain to max velocity - just to ensure velocity <= maximum velocity
  if (abs(velocity) > max_velocity) {
    if (velocity < 0) {
      velocity = -1 * max_velocity;
    }
    else {
      velocity = max_velocity;
    }
  }
  
  //accelerate
  if(turningTime < 0.5) {
    velocity = (turningTime / 0.5) * velocity;
  }
  else {
    accelerated = true;
  }

  lastError = error;
  previousTime = currentTime;
}

void inverse_kinematics() { // gets angular velocity of wheels for turning
  ang_vel[0] = (L + l) * velocity / R_w; // left front
  ang_vel[1] = (L + l) * velocity / R_w; // left rear
  ang_vel[2] = (L + l) * velocity / R_w; // right rear
  ang_vel[3] = (L + l) * velocity / R_w; // right front
}

void set_motor_speed() { // gets motor speed based on angular velocity values
  for (int i = 0; i < 4; i++) {
    motor_speed_Value[i] = ang_vel[i] * 28; // scale angular velocity to motor speed value - COULD BE TUNED BETTER
  }
}

void set_motors() { // sets motor speed
  left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0]);
  left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1]);
  right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2]);
  right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3]);
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

void IR_Sensors() { // Calculates distances from centre of robot to where IR detects
  IR_LONG_1_DIST = LEFT_LONG.getDist() + IR_LONG_OFFSET;
  IR_LONG_2_DIST = RIGHT_LONG.getDist() + IR_LONG_OFFSET;
  IR_MID_1_DIST = LEFT_MID.getDist();
  IR_MID_2_DIST = RIGHT_MID.getDist();
}

void Gyro() { // could be tuned better
  if(first_call) {
    first_call = false;
    prev_gyroTime = millis();
  }
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

void phototransistors() { // gets data from phototransistors
  PT1_reading = analogRead(PT1_pin); // left-most
  PT2_reading = analogRead(PT2_pin); // left-middle
  PT3_reading = analogRead(PT3_pin); // right-middle
  PT4_reading = analogRead(PT4_pin); // right-most
  
  PT_left = PT1_reading + PT2_reading;
  PT_right = PT3_reading + PT4_reading;

  if(PT_left < 250) {
    PT_left = PT_left * 1.2;
  }
  else {
    PT_left = PT_left * 1.05;
  }

  PT_diff = PT_left - PT_right;
  PT_ratio = PT_left / PT_right;
  PT_sum = PT1_reading + PT2_reading + PT3_reading + PT4_reading; // gets sum of all 4 phototransistors
}
//----WRITTEN HELPER FUNCTIONS----
