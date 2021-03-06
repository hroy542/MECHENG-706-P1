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
  FIND_CORNER,
  FORWARD,
  REVERSE,
  STRAFE,
  TURN,
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
float last_est[4] = {0,0,0,0};
float last_var[4] = {999,999,999,999};
float process_noise = 5;
float sensor_noise = 6;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----MA Filter----
#define WINDOW_SIZE 20
int index[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float value[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float SUM[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float FRONT_LIR[WINDOW_SIZE];
float BACK_LIR[WINDOW_SIZE];
float LEFT_MIR[WINDOW_SIZE];
float RIGHT_MIR[WINDOW_SIZE];
float SONAR[5];
float averaged[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
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
const float Kp_gyro = 40;

float radiansAngle = 0;
//----Gyro----

//----Find Corner Flags----
bool parallel = false;
bool LONG = false;
bool SHORT = false;
//----Find Corner Flags----

//----Driving----
bool DRIVING = false;
bool corner_finished = false;
bool is_turning = false;
bool is_driving_middle = false;
//----Driving----

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

float velocity[3] = {0,0,0};
float max_velocity[3] = {25,20,1.5}; // cm/s
float ang_vel[4] = {0,0,0,0};

float motor_speed_Value[4] = {0,0,0,0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {0,0,0};
float currentTime, previousTime, elapsedTime;
float error[3] = {0,0,0};
float lastError[3] = {0,0,0};
float rateError[3] = {0,0,0};

float Pterm[3], Iterm[3], Dterm[3];


// PID VALUES WHEN WE WANT HIGHER CONTROL EFFORT FROM THE Y CONTROLLER RELATIVE TO THE X CONTROLLER
float Kp_original[3] = {2,1.8,1.65};
float Kp_amplified[3] = {2,3.6,1.65};

// PID VALUES FOR X AND Y NEED TO BE TUNED AND TESTED - TURNING SHOULD BE FINE
float Kp[3] = {2,1.8,1.65};
float Ki[3] = {0.06,0.015,0.05};
float Kd[3] = {0,0,0};

float Kp_straight = 40; // SHOULD BE TUNED
float Kp_turn = 1500;
//----PIDValues----

//----Localisation----
const int local_sample_time = 400; //2.5Hz sampling frequency for coordinates
int local_time = 0;
int prev_local_time = 0;
float currentPos[2] = {0,0}; // x,y
//----Localisation----


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
  //initialising
  SerialCom->println("INITIALISING....");
  //delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  static RUN_STATE running_state = FIND_CORNER;

  //FSM
  switch (running_state) {
    case FIND_CORNER:
      running_state = find_corner();
      break;
    //----DRIVING COMPONENT----
    case FORWARD: 
      running_state = forward();
      break;
    case REVERSE:
      running_state = reverse();
      break;
    case STRAFE:
      running_state = strafe();
      break;
    case TURN:
      running_state = turn();
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

//----WRITTEN DRIVING FUNCTIONS----
RUN_STATE find_corner() { // Drives robot to TL or BR corner
  /* 
  Implementation for robot to find TL or BR corner:
  1. If any sensors are very close to wall (i.e. < 5cm), move away from wall.
  2. Rotate robot slowly whilst continuously reading long range IRs
  3. If IRs both detect distance < 60cm (maximum) and are close to one another (10%), stop and align robot to wall - IRs read same distance.
  4. Read ultrasonic to determine distance away from wall - if > 100cm, reverse, otherwise drive forward.
  5. Drive straight using controller until 20cm away from wall.
  6. Rotate 180 degrees (if forwards in step 4.) and align to read distance of other wall (determine whether on short or long side).
  
  LONG
  7. If long side, rotate 90 degrees CCW and align with close wall.
  8. Strafe 5cm left (robot centre 15cm from wall)
  9. Reverse into starting position (15cm from wall) using ultrasonic sensor ensuring alignment using all IRs
  
  SHORT
  7. If short side, reverse 5cm using ultrasonic sensor
  8. Strafe left into starting position (15cm from wall) ensuring alignment using all IRs
  */
  
  find_min_align(); // initial orient of robot parallel to wall
  delay(100);
  
  switch_back_count = 1; // make it so uses the gyro to keep straight
  find_side(); // determines if on long or short side

  delay(100);

  if(LONG) {
    corner_long();
    LONG = false;
  }
  else if(SHORT) {
    corner_short();
    SHORT = false;
  }

  switch_back_count = 0; // reset variables;
  corner_finished = true;

  delay(5000);
  
  return FORWARD;
}

RUN_STATE forward() { // COULD INCREASE Y CONTROLLER GAINS TO KEEP BETTER DISTANCE FROM WALL
  forward_flag = true;
  align_back(); // align mid range IRs to back wall before starting 
  currentAngle = 0;

  delay(100);
  Kp[1] =  Kp_amplified[1];// Assigning the amplified y controller gain
  if(turned) {
    driveXYZ(15, 45 - ((switch_back_count - 5) * 10), 0); 
  }
  else {
    driveXYZ(15, 15 + (switch_back_count * 10), 0); 
  }

  if(switch_back_count == 4) {
    Kp[1] = Kp_original[1];//Reassigning the original y controller gain
    return TURN;
  }
  else {
    Kp[1] = Kp_original[1];//Reassigning the original y controller gain
    return STRAFE;
  }
}

RUN_STATE reverse() { // COULD INCREASE Y CONTROLLER GAINS TO KEEP BETTER DISTANCE FROM WALL 
  forward_flag = false;

  Kp[1] =  Kp_amplified[1];// Assigning the amplified y controller gain
  if(turned) {
    driveXYZ(180, 45 - ((switch_back_count - 5) * 10), 0);
  }
  else {
    driveXYZ(180, 15 + (switch_back_count * 10), 0); 
  }

  if(switch_back_count == 8) {
    Kp[1] = Kp_original[1];//Reassigning the original y controller gain
    return END;
  }
  else {
    Kp[1] = Kp_original[1];//Reassigning the original y controller gain
    return STRAFE;
  }
}

RUN_STATE strafe() { // RESET Y CONTROLLER GAINS 
  switch_back_count++;

  if(turned) {
    if(forward_flag) {
      driveXYZ(15, 45 - ((switch_back_count - 5) * 10), 0);
      return REVERSE;
    } 
    else {
      driveXYZ(180, 45 - ((switch_back_count - 5) * 10), 0);
      return FORWARD;
    }
  }
  else {
    if(forward_flag) {
      driveXYZ(15, 15 + (switch_back_count * 10), 0);
      return REVERSE;
    } 
    else {
      driveXYZ(180, 15 + (switch_back_count * 10), 0);
      return FORWARD;
    }
  }
}

RUN_STATE turn() {
  switch_back_count++;
  turned = true;
  
  driveXYZ(0, 0, 90);
  delay(100);
  
  align();
  delay(100);

  is_driving_middle = true;
  driveXYZ(45, 15, 0);
  is_driving_middle = false;
  delay(100);
  
  align();
  delay(100);
  
  driveXYZ(0, 0, 90);

  return FORWARD;
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
    if(power > 130) {
      power = 130;
    }
    else if(power < -130) {
      power = -130;
    }

    // send power
    left_front_motor.writeMicroseconds(1500 + power);
    left_rear_motor.writeMicroseconds(1500 + power);
    right_rear_motor.writeMicroseconds(1500 + power);
    right_front_motor.writeMicroseconds(1500 + power);
  } while(abs(radiansAngle) < abs(angle)); // exit condition
  
  return;
}

void find_min_align() {
  int count = 0;
  int index = 0;
  float min_dist = 999;

  rotationThreshold = 0;

  while(count < 11) {
    rotate_small(8);

    delay(10);
    
    Ultrasound();

    if(Ultradistance < min_dist) {
      min_dist = Ultradistance;
      index = count;
    }

    count++;
  }

  while((index) < (count - 1)) {
    rotate_small(-8);
    index++;
  }

  rotationThreshold = 1;
  return;
}


void find_side() { // determine whether on short or long side of rectangle

  driveXYZ(20,0,0);

  delay(100);
  
  driveXYZ(0,0,180); // rotate robot to see other wall

  delay(100);

  align_back();

  delay(100);
  Ultrasound();
  //BluetoothSerial.println(Ultradistance);
  
  delay(100);

  // if ultrasonic detects > 150 cm - robot on short side
  if(Ultradistance > 150) {
    LONG = false;
    SHORT = true;
  }
  // if ultrasonic detects < 120 cm - robot on long side
  else if(Ultradistance < 120) {
    SHORT = false;
    LONG = true;
  }

  return;
}

void corner_long() { // drives robot to corner if on long side
  
  driveXYZ(0,0,-90); // rotate 90 degrees CCW
  delay(100);
  
  align(); // might not need
  delay(100);

  driveXYZ(180, 15, 0); // reverse until 15cm from wall (starting position)
  delay(100);

  align(); // NOTE COULD REPLACE ALIGN_BACK() WITH ALIGN() - DEPENDS WHATS MORE RELIABLE
}

void corner_short() { // drives robot to corner if on short side
  //BluetoothSerial.println("short");
  
  driveXYZ(0, 0, -90); // rotate ccw
  delay(100);
  
  driveXYZ(15, 20, 0); // drive straight to wall
  delay(100);
  
  driveXYZ(0, 0, 90); // rotate cw into starting position
  
  align(); // NOTE COULD REPLACE ALIGN_BACK() WITH ALIGN() - DEPENDS WHATS MORE RELIABLE
}
  
void align() { // uses long range IRs to align robot to wall
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.2; // IRs within 0.2cm of one another - SHOULD BE TWEAKED
  int prevTime = millis();

  while(millis() - prevTime < 50) {
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
  }

  while(abs(ir1_dist - ir2_dist) >= alignment_threshold) {
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
    
    if(ir1_dist > ir2_dist) {
      ccw(70);
    }
    else {
      cw(70);
    }
  }

  stop(); // stop motors
  return;
}

void align_back() { //uses mid range IRs to align to wall - COULD ALSO IMPLEMENT BETTER CONTROL (P CONTROL)
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.03;
  int prevTime = millis();

  while(millis() - prevTime < 50) {
    ir1_dist = IR_dist(BACK_LEFT);
    ir2_dist = IR_dist(BACK_RIGHT);
  }

  while(abs(ir1_dist - ir2_dist) >= alignment_threshold) {
    ir1_dist = IR_dist(BACK_LEFT);
    ir2_dist = IR_dist(BACK_RIGHT);
    
    if(ir1_dist > ir2_dist) {
      ccw(70);
    }
    else {
      cw(70);
    }
  }

  stop(); // stop motors
  return;
}

void driveXYZ(float x, float y, float z) { // Drives robot straight in x, y, and z // turning only occurs by itself i.e. no x and y
  DRIVING = true;

  reference[0] = x;
  reference[1] = y;  
  reference[2] = z * (PI/180);

  while(DRIVING) {
    Controller(); // CONTROL LOOP
  }

  is_turning = false;

  stop(); // stop motors
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

void Controller(){
  
  local_time = millis();
 
  gyro();
  Ultrasound();
  IR_Sensors();

  if(corner_finished && (local_time - prev_local_time >= local_sample_time)) {
    if(!turned && !is_turning) {
      currentPos[0] = 200 - Ultradistance;
      currentPos[1] = IR_wall_dist;
    }
    else if(!is_driving_middle && !is_turning) {
      currentPos[0] = Ultradistance;
      currentPos[1] = 120 - IR_wall_dist;
    }
    else if(is_turning){ 
      //do_nothing
    } 
    else if(is_driving_middle) {
      currentPos[0] = 200 - IR_wall_dist;
      currentPos[1] = 120 - Ultradistance;
    }

    BluetoothSerial.print(currentPos[0]);
    BluetoothSerial.print(", ");
    BluetoothSerial.println(currentPos[1]);
    
    prev_local_time = local_time;
  }

  //----Replacement for stuff in DriveXYZ----

  if(reference[0] == 0){
    error[0] = 0;
  }
  else if(reference[0] == 180) {
    // change gains for reversing
    Kp[0] = 1;
    Ki[0] = 0.001;
    
    if(IR_mid_dist < 35) {
      error[0] = IR_mid_dist - 20.0;
    }
    else {
      error[0] = reference[0] - Ultradistance;
    }
  }
  else {
    error[0] = reference[0] - Ultradistance;
  }

  if(reference[1] == 0){
    error[1] = 0;
  }
  else{
    error[1] = reference[1] - IR_wall_dist;
  }

  if(reference[2] == 0){
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
  if((abs(error[0]) + abs(error[1])) <= 2.75 && (reference[2] == 0)) { // XY exit condition - COULD BE TWEAKED ALONGSIDE GAINS
    DRIVING = false;
    return;
  }
  else if(abs(error[2]) < 0.1 && (!(reference[2] == 0))) { // turning exit condition
    DRIVING = false;
    currentAngle = 0;
    return;
  }
  
  delay(8);
}

void PID_Controller(){

  currentTime = millis();                
  elapsedTime = (currentTime - previousTime)/1000.0; 
 
  for (int i = 0; i < 3; i++){

    Pterm[i] = Kp[i] * error[i];
    Iterm[i] += Ki[i] * error[i] * elapsedTime;
    Dterm[i] = Kd[i] * ((error[i] - lastError[i]) / elapsedTime);

    // anti wind-up - Iterm and Pterm (More powerful anti windup - constrains total control effort to always be <= max velocity)
    if(abs(Pterm[i]) > max_velocity[i]) {

      // constrains Iterm such that Pterm + Iterm <= maximum velocity
      if(Pterm[i] < 0) {
        Pterm[i] = (-1 * max_velocity[i]);
      }
      else {
        Pterm[i] = max_velocity[i];
      }
    }
    if(abs(Iterm[i] + Pterm[i]) > max_velocity[i]) {

      // constrains Iterm such that Pterm + Iterm <= maximum velocity
      if((Iterm[i] + Pterm[i]) < 0) {
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
    if(abs(velocity[i]) > max_velocity[i]) {
      if(velocity[i] < 0) {
        velocity [i] = -1 * max_velocity[i];
      }
      else {
        velocity [i] = max_velocity[i];
      }
    }

    lastError[i] = error[i];                                                
  }
  previousTime = currentTime; 
}

void inverse_kinematics(){
  ang_vel[0] = (-velocity[0] + velocity[1] + ((L+l)*velocity[2])) / R_w; // left front
  ang_vel[1] = (-velocity[0] - velocity[1] + ((L+l)*velocity[2])) / R_w; // left rear
  ang_vel[2] = (velocity[0] - velocity[1] + ((L+l)*velocity[2])) / R_w; // right rear
  ang_vel[3] = (velocity[0] + velocity[1] + ((L+l)*velocity[2])) / R_w; // right front
}

void set_motor_speed(){
  for (int i = 0; i < 4; i++){
    motor_speed_Value[i] = ang_vel[i] * 30; // scale angular velocity to motor speed value - COULD BE TUNED BETTER
  }
}

void set_motors() {

  if(switch_back_count == 0 || switch_back_count == 8) { // only use at the edges
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

void Ultrasound(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  Ultraduration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  Ultradistance = ultra_centre_offset + (Ultraduration * 0.034 / 2);
}

void IR_Sensors(){
  // MIGHT NEED TO HAVE A WHILE LOOP TO ENSURE IR DISTANCES REACH STEADY STATE (~50MS) WHEN DRIVEXYZ FIRST CALLED
  
  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);

  IR_MID_1_DIST = IR_dist(BACK_LEFT);
  IR_MID_2_DIST = IR_dist(BACK_RIGHT);
  
  IR_wall_dist = long_centre_offset + (0.37 * IR_LONG_1_DIST + 0.63 * IR_LONG_2_DIST); // distance of centre of robot to wall
  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  
  IR_mid_dist = mid_centre_offset + ((IR_MID_1_DIST + IR_MID_2_DIST) / 2);
  IR_mid_diff = IR_MID_1_DIST - IR_MID_2_DIST;

  if(abs(IR_diff) > 0.5) {
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

  if(angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    angleChange = angularVelocity * (timeElapsed / 1000.0);
    currentAngle += angleChange;
    gyroCorrection = currentAngle * Kp_gyro;
    radiansAngle = currentAngle * (PI/180.0);
  }
  
  delay(5);
  
  prev_gyroTime = gyroTime;
}

float IR_dist(IR code) { // find distances using calibration curve equations
  float est, dist;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      if(adc != 0 && adc <= 650){
        dist = (13391)/(pow(adc,1.172));
        est = Kalman(dist, last_est[0], last_var[0], LEFT_FRONT);
        
        //MA FILTER
        SUM[0] -= FRONT_LIR[index[0]];
        FRONT_LIR[index[0]] = est;
        SUM[0] += est;
        index[0] = (index[0] + 1) % WINDOW_SIZE;
        averaged[0] = SUM[0]/WINDOW_SIZE;
        est = averaged[0];
        last_est[0] = averaged[0];
        //MA FILTER
      } else {
        est = last_est[0];
      }
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      if(adc != 0 && adc <= 650){

      dist = (11178)/(pow(adc,1.14));
      est = Kalman(dist, last_est[1], last_var[1], LEFT_BACK);
      
      //MA FILTER
      SUM[1] -= BACK_LIR[index[1]];
      BACK_LIR[index[1]] = est;
      SUM[1] += est;
      index[1] = (index[1] + 1) % WINDOW_SIZE;
      averaged[1] = SUM[1]/WINDOW_SIZE;
      est = averaged[1];
      last_est[1] = averaged[1];
      //MA FILTER
      } else {
        est = last_est[1];
      }
      break;
    case BACK_LEFT:
      adc = analogRead(IR_MID_1);
      if(adc != 0 && adc <= 650){
      dist = (3730.6)/(pow(adc,1.082));
      est = Kalman(dist, last_est[2], last_var[2], BACK_LEFT);

      //MA FILTER
      SUM[2] -= LEFT_MIR[index[2]];
      LEFT_MIR[index[2]] = est;
      SUM[2] += est;
      index[2] = (index[2] + 1) % WINDOW_SIZE;
      averaged[2] = SUM[2]/WINDOW_SIZE;
      est = averaged[2];
      last_est[2] = averaged[2];
      //MA FILTER
      } else {
        est = last_est[2];
      }
      break;
    case BACK_RIGHT:
      adc = analogRead(IR_MID_2);
      if(adc != 0 && adc <= 650){
      dist = (3491.3)/(pow(adc,1.069));
      est = Kalman(dist, last_est[3], last_var[3], BACK_RIGHT);

      //MA FILTER
      SUM[3] -= RIGHT_MIR[index[3]];
      RIGHT_MIR[index[3]] = est;
      SUM[3] += est;
      index[3] = (index[3] + 1) % WINDOW_SIZE;
      averaged[3] = SUM[3]/WINDOW_SIZE;
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
  
  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  
  switch(code) {
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
//----WRITTEN HELPER FUNCTIONS----
