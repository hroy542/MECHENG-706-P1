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
  REPOSITION,
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

float Kp_align = 100;
float PT_correction = 0;

int repos_time = 0; // time for robot to drive forward to reposition
const int min_detect_threshold = 0; // minimum value to know if fire is present - SUBJECT TO CHANGE 
const int detection_threshold = 1800; // when close to fire - SUBJECT TO CHANGE
//----Phototransistors----

//----Obstacle Avoidance----
bool strafed_left = false;
bool strafed_right = false;
bool strafe_reversed = false;
int strafe_left_time = 0;
int strafe_reverse_left_time = 0;
int strafe_right_time = 0;
int strafe_reverse_right_time = 0;
int pass_start_time = 0;
//----Obstacle Avoidance----

//----Fire Fighting----
const int mosfetPin = 45; // mosfet pin for fan

//########
//int servo_val = 0;
int align_servo_val = 1450; // facing forward
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

//----PIDValues----
float reference = 0;
float currentTime, previousTime, elapsedTime;
float turningTime = 0; // for acceleration when turning
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
    case REPOSITION: // if no fire detected in DETECT, reposition robot
      running_state = reposition();
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
    
    if(PT_sum > 60 && PT_ratio < 1.1) {
      stop();
      break;
    }
    else {
      if(PT_ratio < 1.8) { // slow motors as approaches
        power = 200 * (PT_ratio / 2.5);
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

RUN_STATE reposition() {
  static int power = 100; // Could potentially decrease power depending on how close to obstacle - making stopping less abrupt

  // Get sensor readings
  Ultrasound();
  IR_Sensors();
  Gyro();

  forward_straight(power); // set motor power forward

  // Sweep servo for full range and read PTs
  Sweep_repos();
  phototransistors();

  if(PT_sum > min_detect_threshold) {
    stop();
    delay(100);
    return DETECT; // COULD MAYBE RETURN FORWARD_DEFAULT BASED ON HOW EFECTIVE IT IS AT ALIGNING TOWARDS FIRE
    // return FORWARD_DEFAULT;
  }
  else if(Ultradistance < 10 || IR_LONG_1_DIST < 15 || IR_LONG_2_DIST < 15) { // if obstacle reached - could slow down to a stop 20cm away
    stop();
    delay(100);
    rotate(180);
    return REPOSITION;
  }
  else {
    return REPOSITION;
  }
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
  };

  if(numFires == 1 && extinguished) { // if one fire has just been extinguished - find other fire
    extinguished = false;
    forward_straight(-150);
    delay(500);
    //rotate(180);
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
  static int power = 200; // Could potentially decrease power depending on how close to obstacle - making stopping less abrupt
  
  Ultrasound();
  IR_Sensors();
  //Gyro(); // dont need gyro if using Update()
  Update(); // alignment to fire using PTs and applying gain - TEST
  //Sweep(); // alignment to fire using servo and rotating after each sweep - TEST

  forward_towards_fire(power); // set motor power forward

  // SIDE OBSTACLE DETECTION
  if(IR_MID_1_DIST < 8) {
    stop();
    rotate(20);
    return FORWARD_DEFAULT;
  }
  else if(IR_MID_2_DIST < 8) {
    stop();
    rotate(-20);
    return FORWARD_DEFAULT;
  }

  // FRONT OBSTACLE DETECTION
  if(Ultradistance < 12 || IR_LONG_1_DIST < 15 || IR_LONG_2_DIST < 15) {

//    if(Ultradistance + IR_LONG_1_DIST + IR_LONG_2_DIST < 55) { // if at a wall - DEFINITELY A MORE SOPHISTICATED WAY TO DO THIS
//      stop();
//      at_wall = true;
//      rotate(180);
//      return FORWARD_DEFAULT;
//    }
    
    if(Ultradistance < 10) {
      ultrasonic_close = true;
    }
    else if(IR_LONG_1_DIST < 10) {
      left_IR_close = true;
    }
    else {
      right_IR_close = true;
    }
    
    stop();
    delay(100);
    is_fire_close();
    
    if(fire_is_close) {
      return EXTINGUISH;
    }
    else if(left_IR_close) { // direction of strafe depending on front IR readings
      if(IR_MID_2_DIST < 30) { // if not enough space on the right
        strafe_left_time = millis();
        return STRAFE_LEFT;
      }
      else {
        strafe_right_time = millis();
        return STRAFE_RIGHT;
      }
    }
    else if(right_IR_close) {
      if(IR_MID_1_DIST < 30) { // if not enough space on the left
        strafe_right_time = millis();
        return STRAFE_RIGHT;
      }
      else {
        strafe_left_time = millis();
        return STRAFE_LEFT;
      }
    }
    else { // ultrasonic used to stop
      if(IR_MID_1_DIST > IR_MID_2_DIST) { // strafe left if more space on left side
        strafe_left_time = millis();
        return STRAFE_LEFT;
      }
      else {
        strafe_right_time = millis();
        return STRAFE_RIGHT;
      }
    }
  }
  else {
    return FORWARD_DEFAULT; // keep going forward
  }
}

FIRE_FIGHTING_STATE forward_pass() { // driving forward to pass obstacle
  int power = 200;
  pass_start_time = millis(); // get starting time
  
  while((millis() - pass_start_time) < 1700) { // drive forward until 1500ms elapsed
    IR_Sensors();
    Ultrasound();
    Gyro();

    forward_straight(power); // drive forward

    if(Ultradistance < 12 || IR_LONG_1_DIST < 15 || IR_LONG_2_DIST < 15) { // if obstacle detected

//      if(Ultradistance + IR_LONG_1_DIST + IR_LONG_2_DIST < 55) { // if at a wall - DEFINITELY A MORE SOPHISTICATED WAY TO DO THIS
//        stop();
//        at_wall = true;
//        rotate(180);
//        return FORWARD_DEFAULT;
//      }

      if(Ultradistance < 12) {
        ultrasonic_close = true;
      }
      else if(IR_LONG_1_DIST < 15) {
        left_IR_close = true;
      }
      else {
        right_IR_close = true;
      }
      
      stop();
      delay(100);
      phototransistors();
      is_fire_close();
      
      if(fire_is_close) { // put out fire if detected
        return EXTINGUISH;
      }
      else if(strafed_left) { // strafe left if already strafed left
        return STRAFE_LEFT;
      }
      else if(strafed_right) { // strafe right if already strafed right
        return STRAFE_RIGHT;
      }
    }
  }

  return REALIGN;
}

FIRE_FIGHTING_STATE realign() {
  int power = 150;
  int count = 0;
  bool CCW = false;  
  turret_motor.writeMicroseconds(1445);
  
  if(strafed_left) {                                                                                                                                                                                                
    cw(power);

    while(1) {
      phototransistors();
  
      if(PT_ratio < 1) { // right > left
        PT_ratio = 1.0 / PT_ratio;
      }
      
      if(PT_sum > 80 && PT_ratio < 1.1) {
        stop();
        break;
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

    strafed_left = false;
  }
  else if(strafed_right) {
    ccw(power);

    while(1) {
      phototransistors();
  
      if(PT_ratio < 1) { // right > left
        PT_ratio = 1.0 / PT_ratio;
      }
      
      if(PT_sum > 80 && PT_ratio < 1.1) {
        stop();
        break;
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

    strafed_right = false;
  }

  return FORWARD_DEFAULT;
}

FIRE_FIGHTING_STATE strafe_left() {
  strafed_left = true;
  static int power = -200;
  int reverse_time = 0;
  
  IR_Sensors();
  Gyro();

  strafe(power); // left strafe

  if(IR_MID_1_DIST < 10) { // if left side IR detects obstacle strafe other way to avoid obstacle
    stop();
    strafed_left = false;
    strafe_reversed = true;
    strafe_reverse_right_time = millis() - strafe_left_time; // time needed to strafe back
    return STRAFE_RIGHT;
  }

  if(strafe_reversed) {
    reverse_time = millis();
    while(millis() - reverse_time < strafe_reverse_left_time) {
      Gyro();
      strafe(power);
      delay(5);
    }

    strafe_right_time = millis();
    strafe_reversed = false;
  }

  if(left_IR_close) {
    if(millis() - strafe_left_time < 2300) {
      return STRAFE_LEFT;
    }
    else if(IR_LONG_2_DIST < 20) { // still in the way of obstacle
      delay(100);
      return STRAFE_LEFT;
    }
    else {
      left_IR_close = false;
      return FORWARD_PASS;
    }
  }
  else if(ultrasonic_close) {
    if(millis() - strafe_left_time < 1625) {
      return STRAFE_LEFT;
    }
    else if(IR_LONG_2_DIST < 20) { // still in the way of obstacle
      delay(100);
      return STRAFE_LEFT;
    }
    else {
      ultrasonic_close = false;
      return FORWARD_PASS;
    }
  }
  else if(right_IR_close) {
    if(millis() - strafe_left_time < 950) {
      return STRAFE_LEFT;
    }
    else if(IR_LONG_2_DIST < 20) { // still in the way of obstacle
      return STRAFE_LEFT;
    }
    else {
      right_IR_close = false;
      return FORWARD_PASS;
    }
  }
}

FIRE_FIGHTING_STATE strafe_right() { 
  strafed_right = true;
  static int power = 200;
  int reverse_time = 0;
  
  IR_Sensors();
  Gyro();

  strafe(power); // right strafe

  if(IR_MID_2_DIST < 10) { // if right side IR detects obstacle strafe other way to avoid obstacle
    stop();
    strafed_right = false;
    strafe_reversed = true;
    strafe_reverse_left_time = millis() - strafe_right_time;
    return STRAFE_RIGHT;
  }

  if(strafe_reversed) {
    reverse_time = millis();
    while(millis() - reverse_time < strafe_reverse_right_time) {
      Gyro();
      strafe(power);
      delay(5);
    }

    strafe_right_time = millis();
    strafe_reversed = false;
  }
  
  if(left_IR_close) {
    if(millis() - strafe_right_time < 950) {
      return STRAFE_RIGHT;
    }
    else if(IR_LONG_1_DIST < 20) { // still in the way of obstacle
      delay(100);
      return STRAFE_RIGHT;
    }
    else {
      left_IR_close = false;
      return FORWARD_PASS;
    }
  }
  else if(ultrasonic_close) {
    if(millis() - strafe_right_time < 1625) {
      return STRAFE_RIGHT;
    }
    else if(IR_LONG_1_DIST < 20) { // still in the way of obstacle
      delay(100);
      return STRAFE_RIGHT;
    }
    else {
      ultrasonic_close = false;
      return FORWARD_PASS;
    }
  }
  else if(right_IR_close) {
    if(millis() - strafe_right_time < 2300) {
      return STRAFE_RIGHT;
    }
    else if(IR_LONG_1_DIST < 20) { // still in the way of obstacle
      return STRAFE_RIGHT;
    }
    else {
      right_IR_close = false;
      return FORWARD_PASS;
    }
  }
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
  return;
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

void rotate_small(float angle) { // P Control rotate function for small angles
  float power;

  angle = angle * (PI / 180);

  currentAngle = 0;

  do {
    Gyro();
    power = (angle - radiansAngle) * Kp_turn;

    // constrain
    if (power > 120) {
      power = 120;
    }
    else if (power < -120) {
      power = -120;
    }

    // send power
    left_front_motor.writeMicroseconds(1500 + power);
    left_rear_motor.writeMicroseconds(1500 + power);
    right_rear_motor.writeMicroseconds(1500 + power);
    right_front_motor.writeMicroseconds(1500 + power);
  } while (abs(radiansAngle) < abs(angle)); // exit condition

  return;
}

void Update() { // FIRE ALIGNMENT IMPLEMENTATION 1

  phototransistors();

  if(PT_ratio < 1) { // right > left
    PT_ratio = 1.0 / PT_ratio;
    PT_correction = Kp_align * (PT_ratio - 1);
  }
  else {
    PT_correction = -Kp_align * (PT_ratio - 1);
  }

  if(PT_correction > 100) { // constrain to within 100
    PT_correction = 100;
  }
  else if(PT_correction < -100) {
    PT_correction = -100;
  }
}

void Sweep() { // FIRE ALIGNMENT IMPLEMENTATION 2
  static int servo_val = 900;
  static int aligned_servo_val = 0;
  static float min_PT_ratio = 999;
  static bool CCW = true;
  static int ccw_val = 1800;
  static int cw_val = 1200;
  phototransistors();

  if(PT_ratio < 1) { // ensures PT_ratio is always >= 1
    PT_ratio = 1.0 / PT_ratio;
  }

  if(PT_ratio - 1 < min_PT_ratio) { // if phototransistors ratio lower, update min ratio and servo value
    min_PT_ratio = PT_ratio - 1;
    aligned_servo_val = servo_val; // updates aligned servo position
  }

  if(CCW) { // sweep ccw
    servo_val += 100;
    if(servo_val == ccw_val) {
      CCW = false;
      stop();
      rotate((1500 - aligned_servo_val) / 10); // reorient towards fire - MOST LIKELY NEED TO ADJUST GAINS (ANGLE TOO SMALL)
    }
  }
  else if(!CCW) { // sweep cw
    servo_val -= 100;
    if(servo_val == cw_val) {
      CCW = true;
      stop();
      rotate((1500 - aligned_servo_val) / 10); // reorient towards fire - MOST LIKELY NEED TO ADJUST GAINS (ANGLE TOO SMALL)
    }
  }
  
  turret_motor.writeMicroseconds(servo_val); // set turret angle
  delay(50); // might need to use sampling
}

void Sweep_repos() { // Sweeps fan when repositioning if fire not detected
  static int servo_val = 1500;
  static int aligned_servo_val = 0;
  static float min_PT_ratio = 999;
  static bool CCW = true;
  static int ccw_val = 2100;
  static int cw_val = 900;

  if(CCW) { // sweep ccw
    servo_val += 300;
    if(servo_val == ccw_val) {
      CCW = false;
    }
  }
  else if(!CCW) { // sweep cw
    servo_val -= 300;
    if(servo_val == cw_val) {
      CCW = true;
    }
  }
  
  turret_motor.writeMicroseconds(servo_val); // set turret angle
  delay(50); // might need to use sampling
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
  
  while(abs(PT2_reading - PT3_reading) > 40) { // read middle phototransistors
    if(PT2_reading > PT3_reading) { // ccw
      align_servo_val += 10; // increment ccw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
    else { // cw
      align_servo_val -= 10; // increment cw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
    
    delay(100);//****** was 10 before
    phototransistors(); // read phototransistors
  }
  
  align_servo_val = turret_motor.read();
}

void align_robot() { // aligns robot to fire using PTs
  int aligned_servo_val = 1450;
    
  turret_motor.writeMicroseconds(aligned_servo_val);
  phototransistors();

  while(abs(PT_left - PT_right) > 10) { // read middle phototransistors
    if(PT_left > PT_right) { // ccw
      ccw(80);
    }
    else { // cw
      cw(80);
    }
    delay(20);
    
    phototransistors(); // read phototransistors
  }
}

void put_out_fire() {
  // Set mosfet pin high for 10 seconds (fan)
  turret_motor.writeMicroseconds(align_servo_val);
  phototransistors(); 

  while((PT2_reading >= 250)&&(PT3_reading >= 250)){
    digitalWrite(mosfetPin, HIGH);
    phototransistors();
    delay(50);
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
  left_front_motor.writeMicroseconds(1500 + speedval + PT_correction);
  left_rear_motor.writeMicroseconds(1500 + speedval + PT_correction);
  right_rear_motor.writeMicroseconds(1500 - speedval + PT_correction);
  right_front_motor.writeMicroseconds(1500 - speedval + PT_correction);
}

void forward_straight(int speedval) // drives forward straight
{
  // remove gyroCorrection and add PT_correction if using closed loop alignement
  left_front_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
}

void strafe(float speedval) // +ve = right, -ve = left correcting for angular rotation and x direction
{
  left_front_motor.writeMicroseconds(1500 + speedval - (speedval/4.4) - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 - speedval - (speedval/4.4) - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - speedval + (speedval/4.4) - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + speedval + (speedval/4.4) - gyroCorrection);
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
  IR_LONG_1_DIST = IR_dist(FRONT_LEFT) + IR_LONG_OFFSET;
  IR_LONG_2_DIST = IR_dist(FRONT_RIGHT) + IR_LONG_OFFSET;
  IR_MID_1_DIST = IR_dist(LEFT);
  IR_MID_2_DIST = IR_dist(RIGHT);
}

void Gyro() { // could be tuned better
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
      adc = analogRead(IR_MID_1);
      if (adc != 0 && adc <= 650) {
        dist = (5586.2) / (pow(adc, 1.15));
        est = dist;
        //est = Kalman(dist, last_est[0], last_var[0], LEFT);

        //MA FILTER
//        SUM[0] -= FRONT_LIR[index[0]];
//        FRONT_LIR[index[0]] = est;
//        SUM[0] += est;
//        index[0] = (index[0] + 1) % WINDOW_SIZE;
//        averaged[0] = SUM[0] / WINDOW_SIZE;
//        est = averaged[0];
//        last_est[0] = averaged[0];
//        //MA FILTER
//      } else {
//        est = last_est[0];
      }
      else {
        est = 999;
      }
      break;
    case RIGHT:
      adc = analogRead(IR_MID_2);
      if (adc != 0 && adc <= 650) {
        dist = (452.63) / (pow(adc, 0.728));
        est = dist;
//        est = Kalman(dist, last_est[1], last_var[1], RIGHT);
//
//        
//        //MA FILTER
//        SUM[1] -= BACK_LIR[index[1]];
//        BACK_LIR[index[1]] = est;
//        SUM[1] += est;
//        index[1] = (index[1] + 1) % WINDOW_SIZE;
//        averaged[1] = SUM[1] / WINDOW_SIZE;
//        est = averaged[1];
//        last_est[1] = averaged[1];
//        //MA FILTER
//      } else {
//        est = last_est[1];
      }
      else {
        est = 999;
      }
      break;
    case FRONT_LEFT:
      adc = analogRead(IR_LONG_1);
      if (adc != 0 && adc <= 650) {
        dist = (7058.6) / (pow(adc, 1.066));
        est = dist;
//        est = Kalman(dist, last_est[2], last_var[2], FRONT_LEFT);
//
//        //MA FILTER
//        SUM[2] -= LEFT_MIR[index[2]];
//        LEFT_MIR[index[2]] = est;
//        SUM[2] += est;
//        index[2] = (index[2] + 1) % WINDOW_SIZE;
//        averaged[2] = SUM[2] / WINDOW_SIZE;
//        est = averaged[2];
//        last_est[2] = averaged[2];
//        //MA FILTER
//      } else {
//        est = last_est[2];
      }
      else {
        est = 999;
      }
      break;
    case FRONT_RIGHT:
      adc = analogRead(IR_LONG_2);
      if (adc != 0 && adc <= 650) {
        dist = (11178) / (pow(adc, 1.14));
        est = dist;
//        est = Kalman(dist, last_est[3], last_var[3], FRONT_RIGHT);
//
//        //MA FILTER
//        SUM[3] -= RIGHT_MIR[index[3]];
//        RIGHT_MIR[index[3]] = est;
//        SUM[3] += est;
//        index[3] = (index[3] + 1) % WINDOW_SIZE;
//        averaged[3] = SUM[3] / WINDOW_SIZE;
//        est = averaged[3];
//        last_est[3] = averaged[3];
//        //MA FILTER
//      } else {
//        est = last_est[3];
      }
      else {
        est = 999;
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

void phototransistors() { // gets data from phototransistors
  PT1_reading = analogRead(PT1_pin); // left-most
  PT2_reading = analogRead(PT2_pin); // left-middle
  PT3_reading = analogRead(PT3_pin); // right-middle
  PT4_reading = analogRead(PT4_pin); // right-most
  
  PT_left = PT1_reading + PT2_reading;
  PT_right = PT3_reading + PT4_reading;

  if(PT_left < 200) {
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
