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
  REPOSITION,//NEW**
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
const float ultra_centre_offset = 3.5;
//----Ultrasound----

//----IR----
enum IR {
  FRONT_LEFT, // mid
  FRONT_RIGHT, // mid
  LEFT, // long
  RIGHT, // long
};

// Left long range IR
const int IR_LONG_1 = A4;
float IR_LONG_1_DIST = 0;

// Right long range IR
const int IR_LONG_2 = A5;
float IR_LONG_2_DIST = 0;

const float IR_LONG_OFFSET = 7; // distance from IR to centre of robot //NEW**

// Front left mid range IR
const int IR_MID_1 = A6;
float IR_MID_1_DIST = 0;

// Front right mid range IR
const int IR_MID_2 = A7;
float IR_MID_2_DIST = 0;

const float IR_MID_OFFSET = 4; // distance from IR to centre of robot
//----IR----

//----IR Kalman Filter----
float last_est[4] = {0, 0, 0, 0};
float last_var[4] = {999, 999, 999, 999};
float process_noise = 5;
float sensor_noise = 6;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//NEW**
//----Boolean variables for which sensor detects obstacle----
bool left_IR_close = false; 
bool right_IR_close = false;
bool ultrasonic_close = false;
//----Boolean variables for which sensor detects obstacle----
//NEW**

//----Phototransistor----
//enum PT { // might not need
//  PT1, // left most
//  PT2, // left centre
//  PT3, // right centre
//  PT4, // right most
//};

const int PT1_pin = A9;
const int PT2_pin = A10;
const int PT3_pin = A12;
const int PT4_pin = A13;

int PT1_reading = 0;
int PT2_reading = 0;
int PT3_reading = 0;
int PT4_reading = 0;
int PT_left = 0;
int PT_right = 0;

int PT_diff = 0;
float PT_ratio = 0;
int PT_sum = 0;

float Kp_align = 100;
float PT_correction = 0;

int repos_time = 0; // time for robot to drive forward to reposition
const int min_detect_threshold = 0; // minimum value to know if fire is present - SUBJECT TO CHANGE 
const int detection_threshold = 2000; // SUBJECT TO CHANGE
//----Phototransistor----

//----Obstacle Avoidance----
bool strafed_left = false;
bool strafed_right = false;
bool strafe_reversed = false;
int strafe_left_time = 0;
int strafe_right_time = 0;
int pass_start_time = 0;
int strafe_back_time = 0;
//----Obstacle Avoidance----

//----Fire Fighting----
const int mosfetPin = 45; // mosfet pin for fan

int servo_val = 0;
int align_servo_val = 1500;

int numFires = 0;
bool out_of_detect = true;
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

float Kp_turn = 1250;
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

//  static RUN_STATE running_state = FIND_FIRE;

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

  int servo_val = 900;
  int servo_max = 0;
  int max_sum = 0;  
  int temp_sum;
  int rotation_count = 1;
  int max_rotation_count = 1;
  int count = 0;
  int rotation_back = 0;
  int minimum = 100;  // MINIMUM VALUE THAT WE CAN COMFORTABLY SAY IS FACING FIRE AND IS NOT TOO FAR - SUBJECT TO CHANGE

  
  ////-------FINN'S NEW STUFF-------
  open_rotate(150);
  phototransistors();
//   temp_sum = PT_sum;
//   while (temp_sum > (max_sum - 50)){
//       max_sum = temp_sum;
//       phototransistors();
//       temp_sum = PT_sum;
//   }

//   open_rotate(0);
  max_sum = (PT_sum > max_sum)? PT_sum : max_sum;
  while (PT_sum > (max_sum - 50)){
      phototransistors();
      max_sum = (PT_sum > max_sum)? PT_sum : max_sum;
  }
  stop();
  out_of_detect = true;
  Serial.println("Leaving Detect");
  return FIND_FIRE;
  
  ////-------FINN'S NEW STUFF-------

//  while(rotation_count <= 3) { // for 3 robot orientations
//    while(count < 15) { // full range rotation
//      
//      if(rotation_count % 2 == 1) {
//        servo_val += (1200 / 15); // 600 is range from centre, 10 is number of increments
//      }
//      else {
//        servo_val -= (1200 / 15);
//      }
//      
//      turret_motor.writeMicroseconds(servo_val); // sets servo position
//      phototransistors();
//  
//      if(PT_sum > max_sum) { // updates the maximum values for servo and phototransistors
//        BluetoothSerial.println("Sum Updated");
//        servo_max = servo_val;
//        max_sum = PT_sum;
//        max_rotation_count = rotation_count;
//      }
//
//      count++;
//      delay(200);
//    }
//    
//    // IF WE KNOW FIRE HAS BEEN DETECTED AND IS FAIRLY CLOSE - BREAK LOOP AND SAVE TIME
//    if(max_sum > minimum) {
//      break;
//    }
//    currentAngle = 0;
//    radiansAngle = 0;
//    rotate(120); // rotate robot 120 degrees
//    count = 0;
//    rotation_count++;
//  }
//
//  if(max_sum <= min_detect_threshold) { // if no fire detected 
//    repos_time = millis();
//    return REPOSITION;
//  }
//  else {
//    turret_motor.writeMicroseconds(1500); // reset to default
////     rotate(120 * ((max_rotation_count - 1) % 3) - ((servo_max - 1500) / 10)); // orients robot to face fire
////     currentAngle = 0;
//    
//    rotation_back = 120 * ((max_rotation_count - 1) % 3) - ((servo_max - 1500) / 10);
//    if(rotation_back > 180) {
//      rotation_back = rotation_back - 360;
//    }
//
//    rotate(rotation_back); // orients robot to face fire
//    currentAngle = 0; // reset current angle for gyro
//    
//    out_of_detect = true;
//    return FIND_FIRE;
//  }
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
  }
  else if(Ultradistance < 10 || IR_MID_1_DIST < 10 || IR_MID_2_DIST < 10) { // if obstacle reached - could slow down to a stop 20cm away
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

//  if(out_of_detect) { // if just came out of detect state
//    align_robot(); // align robot towards fire
//    out_of_detect = false;
//  }

  //FSM for fire fighting
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

  if(numFires == 1 && extinguished) { // if one fire has just been extinguished - find other fire
    forward_straight(-150);
    delay(500);
    forward_straight(0);
    rotate(180);
    return DETECT;
    Serial.print("Here");
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
  Serial.println("In Forward default");
  Ultrasound();
  IR_Sensors();
  Gyro();
  //Update(); // alignment to fire using PTs and applying gain

 // bluetoothcall(-1, -1, -1, -1, Ultradistance, IR_MID_1_DIST, IR_MID_2_DIST);
  //Sweep();  // alignment to fire using servo and rotating after each sweep

  forward_straight(power); // set motor power forward

  // SIDE OBSTACLE DETECTION - MIGHT SWITCH TO MID RANGE IRS
  if(IR_LONG_1_DIST < 11) {
    rotate(20);
    return FORWARD_DEFAULT;
  }
  else if(IR_LONG_2_DIST < 11) {
    rotate(-20);
    return FORWARD_DEFAULT;
  }
  
  
  // FRONT OBSTACLE DETECTION
  if(Ultradistance < 20 || IR_MID_1_DIST < 20 || IR_MID_2_DIST < 20) {

    if(Ultradistance + IR_MID_1_DIST + IR_MID_2_DIST < 45) { // if at a wall - DEFINITELY A MORE SOPHISTICATED WAY TO DO THIS
      rotate(180);
      return FORWARD_DEFAULT;
    }
    
    if(Ultradistance < 10) {
      ultrasonic_close = true;
    }
    else if(IR_MID_1_DIST < 10) {
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
      left_IR_close = false;
      if(IR_LONG_2_DIST < 43) { // if not enough space on the right
        return STRAFE_LEFT;
      }
      else {
        return STRAFE_RIGHT;
      }
    }
    else if(right_IR_close) {
      right_IR_close = false;
      if(IR_LONG_1_DIST < 43) { // if not enough space on the left
        return STRAFE_RIGHT;
      }
      else {
        return STRAFE_LEFT;
      }
    }
    else { // ultrasonic used to stop
      ultrasonic_close = false;
      if(IR_LONG_1_DIST > IR_LONG_2_DIST) { // strafe left if more space on left side
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
  int power = 150;
  pass_start_time = millis(); // get starting time
  
  while((millis() - pass_start_time) < 1500) { // drive forward until 1500ms elapsed
    IR_Sensors();
    Ultrasound();
    Gyro();
    bluetoothcall(-1, -1, -1, -1, Ultradistance, IR_MID_1_DIST, IR_MID_2_DIST);

    forward_straight(power); // drive forward

    if(Ultradistance < 10 || IR_MID_1_DIST < 10 || IR_MID_2_DIST < 10) { // if obstacle detected

      if(Ultradistance + IR_MID_1_DIST + IR_MID_2_DIST < 45) { // if at a wall - DEFINITELY A MORE SOPHISTICATED WAY TO DO THIS
        rotate(180);
        return FORWARD_DEFAULT;
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

  strafe_back_time = millis(); // get starting time for strafe back

  // CAN EITHER STRAFE BACK IN LINE OR RETURN FORWARD_DEFAULT IF USING CLOSED LOOP CONTROL TO FACE FIRE
  // strafe back to position 
  if(strafed_right) {
    return STRAFE_LEFT;
  }
  else if(strafed_left) {
    return STRAFE_RIGHT;
  }
  //return FORWARD_DEFAULT;
}

FIRE_FIGHTING_STATE strafe_left() {
  static int power = -150;
  IR_Sensors();
  Gyro();

  strafe(power); // left strafe
  bluetoothcall(-1, -1, -1, -1, Ultradistance, IR_MID_1_DIST, IR_MID_2_DIST);

  if(IR_LONG_1_DIST < 15) { // if left side IR detects obstacle strafe other way to avoid obstacle
    stop();
    strafe_reversed = true;
    return STRAFE_RIGHT;
  }

  if(IR_MID_2_DIST < 30) { // keep strafing if obstacle in the way
    strafe_left_time = millis(); // update time until obstacle passed
    return STRAFE_LEFT;
  }
  else if(strafed_right) { // if already strafed right to avoid obstacle
    if(millis() - strafe_back_time > 500) { // strafe left for 500ms (will need to be adjusted) then go forward
      strafed_right = false;
      stop();
      return FORWARD_DEFAULT;
    }
    else {
      return STRAFE_LEFT;
    }
  }
  else {
    if(millis() - strafe_left_time > 500) { // continue strafing left for 300ms (needs to be tweaked) once obstacle passed - to ensure cleared
      strafed_left = true;
      stop();
      return FORWARD_PASS;
    }
    else {
      return STRAFE_LEFT; // continue strafing left
    }
  }
}

FIRE_FIGHTING_STATE strafe_right() { 
  static int power = 150;
  IR_Sensors();
  Gyro();
  bluetoothcall(-1, -1, -1, -1, Ultradistance, IR_MID_1_DIST, IR_MID_2_DIST);

  strafe(power);

  if(IR_LONG_2_DIST < 15) { // if right side IR detects obstacle strafe other way to avoid obstacle
    stop();
    strafe_reversed = true;
    return STRAFE_LEFT;
  }

  if(IR_MID_1_DIST < 30) { // keep strafing if obstacle in the way
    strafe_right_time = millis(); // update time until obstacle passed
    return STRAFE_RIGHT;
  }
  else if(strafed_left) { // if already strafed right to avoid obstacle
    if(millis() - strafe_back_time > 500) { // strafe right for 500ms then go forward again
      strafed_left = false;
      stop();
      return FORWARD_DEFAULT;
    }
    else {
      return STRAFE_RIGHT;
    }
  }
  else {
    if(millis() - strafe_right_time > 500) { // continue strafing left for 300ms once obstacle passed - to ensure cleared
      strafed_right = true;
      stop();
      return FORWARD_PASS;
    }
    else {
      return STRAFE_RIGHT;
    }
  }
}

FIRE_FIGHTING_STATE extinguish() { // extinguish fire state
  numFires++; // increment number of fires extinguished
  Serial.println("In Extinguish");
  align_fan();
  put_out_fire();

  extinguished = true;
  fire_is_close = false;
  return FORWARD_DEFAULT;
}

void rotate(float angle) { // Rotates robot using PID control - MIGHT NEED TO ADJUST GAINS DEPENDING ON ANGLE
  TURNING = true;
  
  if(angle < 45) { // For small angles (i.e. when realigning to face fire after sweeping) change gains - NEEDS TO BE TUNED (if doesn't work well use rotate_small function)
    Kp = 8;
    Ki = 0.1;
  }
  
  reference = angle * (PI / 180); // convert to radians

  while (TURNING) {
    TurnController(); // CONTROL LOOP
  }
  
  Kp = 2;
  Ki = 0.1;
  turningTime = 0;
  currentAngle = 0;
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
}

void Sweep() { // FIRE ALIGNMENT IMPLEMENTATION 2
  static int servo_val = 900; // CHANGED FROM 1500 to 900
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
  delay(50);
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
  BluetoothSerial.println("HERE");
  int prev_pos = turret_motor.read(); // read where servo is currently
  int servo_val = 900;
  turret_motor.writeMicroseconds(servo_val); // reposition to centre - SHOULD PROBABLY DO A FULL SWEEP
  phototransistors();

  while(servo_val < 2100) { // sweep servo to see if fire is close
    phototransistors();//*****
    BluetoothSerial.println("Sweep LOOP in is Fire Close?");
    //bluetoothcall(PT1_reading, PT2_reading, PT3_reading, PT4_reading, Ultradistance, -1, -1);

    if(PT_sum > detection_threshold) { // if greater than threshold - i.e. in range - return
      fire_is_close = true;
      BluetoothSerial.println("PTSum Threshhold Check");

      return;
    }

    servo_val += 100;
    turret_motor.writeMicroseconds(servo_val); // Missing in Leighton's Script, keeping it here
    delay(200);
  }
  
  turret_motor.writeMicroseconds(prev_pos); // set turret position to what is was before entering function
}

void align_fan() {
  servo_val = turret_motor.read();
//  Serial.println(servo_val);
  turret_motor.writeMicroseconds(align_servo_val);
  servo_val = turret_motor.read();
//  Serial.println(servo_val);

  BluetoothSerial.println("IN ALIGN FAN"); 
  phototransistors();

  while(abs(PT2_reading - PT3_reading) > 45) { // read middle phototransistors
    Serial.println("ALIGN FAN WHILE LOOP"); 
    if(PT2_reading > PT3_reading) { // ccw
      align_servo_val += 20; // increment ccw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
    else { // cw
      align_servo_val -= 20; // increment cw servo position
      turret_motor.writeMicroseconds(align_servo_val);
    }
//    BluetoothSerial.println("Align Fan Loop");
    delay(190);//****** was 10 before
    phototransistors(); // read phototransistors
    bluetoothcall(PT1_reading, PT2_reading, PT3_reading, PT4_reading, -1, -1, -1);

  }
  BluetoothSerial.println("LEAVING ALIGN FAN");
  servo_val = turret_motor.read();
//  Serial.println(servo_val);
}

void align_robot() { // aligns robot to fire using PTs
  int aligned_servo_val = 1500;
    
  turret_motor.writeMicroseconds(aligned_servo_val);
  phototransistors();

  while(abs(PT_left - PT_right) > 30) { // read middle phototransistors
     if(PT_left > PT_right) { // ccw
      ccw(69);
    }
    else { // cw
      cw(69);
    }
    delay(20);
    
    phototransistors(); // read phototransistors
  }
}

void put_out_fire() {
//  Serial.println(servo_val);
  turret_motor.writeMicroseconds(align_servo_val);
  BluetoothSerial.println("IN PUT OUT FIRE");
  phototransistors(); 
// Serial.println(PT2_reading);
// Serial.println(PT3_reading);

//    while((PT2_reading >= 300)&&(PT3_reading >= 300)){
    while((PT3_reading >= 300)){
      digitalWrite(mosfetPin, HIGH);
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

void open_rotate(int speedval)
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
// void forward(int speedval) // SHOULD PROBABLY IMPLEMENT ACCELERATION - LOW PRIORITY
{
    // remove gyroCorrection and add PT_correction if using closed loop alignement
  left_front_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
}

void strafe(int speedval) // +ve = right, -ve = left. SHOULD PROBABLY IMPLEMENT ACCELERATION
{
  left_front_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
  left_rear_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
  right_rear_motor.writeMicroseconds(1500 - speedval - gyroCorrection);
  right_front_motor.writeMicroseconds(1500 + speedval - gyroCorrection);
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
    currentAngle = 0;
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
  IR_LONG_1_DIST = IR_LONG_OFFSET + IR_dist(LEFT);
  IR_LONG_2_DIST = IR_LONG_OFFSET + IR_dist(RIGHT);
  IR_MID_1_DIST = IR_MID_OFFSET + IR_dist(FRONT_LEFT);
  IR_MID_2_DIST = IR_MID_OFFSET + IR_dist(FRONT_RIGHT);
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

float IR_dist(IR code) { // FILTERING NEEDS TO BE REDONE
  float est, dist;
  int adc;

  switch (code) {
    case LEFT:
      adc = analogRead(IR_LONG_1);
//      BluetoothSerial.print(adc);
      if (adc != 0 && adc <= 650) {
        dist = (16537) / (pow(adc, 1.219));
        est = dist;
//        est = Kalman(dist, last_est[0], last_var[0], LEFT);
//
//        //MA FILTER
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
      break;
    case RIGHT:
      adc = analogRead(IR_LONG_2);
//      BluetoothSerial.print(adc);
      if (adc != 0 && adc <= 650) {
        dist = (2427) / (pow(adc, 0.888));
        est = dist;
//        est = Kalman(dist, last_est[1], last_var[1], RIGHT);
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
      break;
    case FRONT_LEFT:
      adc = analogRead(IR_MID_1);
      BluetoothSerial.println(adc);
      if (adc != 0 && adc <= 650) {
        dist = (1995.7) / (pow(adc, 0.969));
        est = dist;
//        est = Kalman(dist, last_est[2], last_var[2], FRONT_LEFT);

        //MA FILTER
//        SUM[2] -= LEFT_MIR[index[2]];
//        LEFT_MIR[index[2]] = est;
//        SUM[2] += est;
//        index[2] = (index[2] + 1) % WINDOW_SIZE;
//        averaged[2] = SUM[2] / WINDOW_SIZE;
//        est = averaged[2];
//        last_est[2] = averaged[2];
        //MA FILTER
//      } else {
//        est = last_est[2];
      }
      break;
    case FRONT_RIGHT:
      adc = analogRead(IR_MID_2);
      BluetoothSerial.println(adc);
      if (adc != 0 && adc <= 650) {
        dist = (3805.7) / (pow(adc,1.084));
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
  
  PT_left = PT1_reading + PT2_reading;
  PT_right = PT3_reading + PT4_reading;

  PT_diff = PT_left - PT_right;
  PT_ratio = PT_left / PT_right;
  PT_sum = PT1_reading + PT2_reading + PT3_reading + PT4_reading; // gets sum of all 4 phototransistors
}

void bluetoothcall(int PT1, int PT2, int PT3, int PT4, float US, float IRL, float IRR) {
  String Delimiter = ", ";

  if(PT1 != -1) {
    BluetoothSerial.print("PT1: ");
    BluetoothSerial.print(PT1);
    BluetoothSerial.print(Delimiter);
  } 
  if(PT2 != -1) {
    BluetoothSerial.print("PT2: ");
    BluetoothSerial.print(PT2);
    BluetoothSerial.print(Delimiter);
  } 
  if(PT3 != -1) {
    BluetoothSerial.print("PT3: ");
    BluetoothSerial.print(PT3);
    BluetoothSerial.print(Delimiter);
  } 
  if(PT4 != -1) {
    BluetoothSerial.print("PT4: "); 
    BluetoothSerial.print(PT4);
    BluetoothSerial.print(Delimiter);
  } 
  if(US != -1) {
    BluetoothSerial.print("US: ");
    BluetoothSerial.print(US);
    BluetoothSerial.print(Delimiter);
  } 
  if(IRL != -1) {
    BluetoothSerial.print("IRL: ");
    BluetoothSerial.print(IRL);
    BluetoothSerial.print(Delimiter);
  }
  BluetoothSerial.print("IRR: ");
  BluetoothSerial.println(IRR); 
}
//----WRITTEN HELPER FUNCTIONS----
