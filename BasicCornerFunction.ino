// SAMPLING FOR ULTRA AND PID - DELAYS MIGHT BE BETTER
#include <Servo.h>  //Need for Servo pulse output

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};


//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;


//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;
float Ultraduration;
const float ultra_centre_offset = 11.0;

//const int ultra_sampling_time = 40; //40ms
//int ultra_time;
//int prev_ultra_time;
//bool ultra_first_call = true;
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
float sensor_noise = 5;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----MA Filter----
#define WINDOW_SIZE 13
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
float rotationThreshold = 3;      // because of gyro drifting, defining rotation angular velocity less  
float angularVelocity = 0;
                                                       // than this value will be ignored 
int timeElapsed = 0;
int prev_gyroTime = 0;
int gyroTime = 0;
float gyroRate = 0;                      // read out value of sensor in voltage   
float angleChange = 0;
float currentAngle = 0;               // current angle calculated by angular velocity integral on  

float radiansAngle = 0;
//----Gyro----

//----Go to Corner Variables----
bool parallel = false;
bool LONG = false;
bool SHORT = false;
//----Go to Corner Variables----

//----Driving----
bool DRIVING = false;
bool ignore_x = false;
bool ignore_y = false;
bool ignore_z = false;
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

//StraightLine
float Kp_r[3] = {2,2.5,1.65};
float Ki_r[3] = {0.018,0.02,0.05};
float Kd_r[3] = {0,0,0};

float Kp_straight = 80;
float Kp_turn = 500;
float Kp_align = 80; // gain for aligning to wall

//const int pid_sample_time = 40; // 40ms sampling period - 25Hz
//bool pid_first_call = true;
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
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}

STATE running() {

  find_corner();

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {

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
void find_corner() { // Drives robot to TL or BR corner
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
//  orient(); // initial orient of robot parallel to wall
//
//  delay(500);
//  
//  float wall_dist = (0.63 * IR_dist(LEFT_FRONT)) + (0.37 * IR_dist(LEFT_BACK));
//
//  driveXYZ(25.0, wall_dist, 0);
//
//  delay(500);
//
//  find_side(); // determines if on long or short side
//
//  delay(500);
//
//  if(LONG) {
//    corner_long();
//    LONG = false;
//  }
//  else if(SHORT) {
//    corner_short();
//    SHORT = false;
//  }
//
//  disable_motors();




//  driveXYZ(20,20,0);
//
//  delay(500);
//
//  driveXYZ(20,30,0);
//
//  delay(500);
//
//  driveXYZ(50,30,0);
//
//  delay(500);
//
//  driveXYZ(50,20,0);
//
//  delay(500);
//  
//  driveXYZ(0,0,-PI/2);
//
//  delay(500);
//
//  driveXYZ(0,0,PI/2);
//
//  delay(500);

  // change FSM state - in full code
}

void orient() { // initial orienting of robot parallel to wall using IR
  float ir1_dist, ir2_dist, ratio;
  
  // while robot is not parallel to wall
  while(!parallel) {
    ccw(90); // rotate CCW
    
    // get distances for both IRs
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
    
    // if both IRs are within 55cm
    if(ir1_dist < 55 && ir2_dist < 55) {
      
      // calculate ratio between IR distances
      if(ir1_dist < ir2_dist) {
        ratio = ir1_dist / ir2_dist;
      }
      else {
        ratio = ir2_dist / ir1_dist;
      }
      
      // if ratio is ~1 (i.e. nearly parallel) stop
      if(ratio >= 0.95) {
        stop();
        delay(100);
       
        align(); // align to ensure fully parallel
        parallel = true; 
      }
    }
  }

  return;
}

void find_side() { // determine whether on short or long side of rectangle
  float ultra_dist;
  
  driveXYZ(0,0,PI); // rotate robot to see other wall

  delay(100);

  align_back();

  delay(100);

  ultra_dist = Ultrasound();
  //ultra_first_call = true;

  // if ultrasonic detects > 160 cm - robot on short side
  if(ultra_dist > 150) {
    LONG = false;
    SHORT = true;
  }
  // if ultrasonic detects < 130 cm - robot on long side
  else if(ultra_dist < 130) {
    SHORT = false;
    LONG = true;
  }

  return;
}

void corner_long() { // drives robot to corner if on long side
  float ultra_dist;
  
  driveXYZ(0,0,-PI/2); // rotate 90 degrees CCW
  //drive(0, 0, -PI/2);

  delay(100);

  ultra_dist = Ultrasound(); // read distance from wall
  //ultra_first_call = true;
  
  driveXYZ(ultra_dist, 15, 0); // strafe left until 15cm from wall
  delay(100);
  
  driveXYZ(185, 15, 0); // reverse until 15cm from wall (starting position)
  delay(100);

  align();
}

void corner_short() { // drives robot to corner if on short side
  driveXYZ(185, 0, 0); // reverse until 15cm from wall
  delay(100);
  align();

  delay(100);
  
  driveXYZ(185, 15, 0); // strafe left into starting position
  delay(100);
  align();
}
  
void align_controller() { // uses long range IRs with gain to align robot to wall - P CONTROLLER
  float ir1_dist, ir2_dist, diff, power;
  float alignment_threshold = 0.3;

  // read long range IRs
  ir1_dist = IR_dist(LEFT_FRONT);
  ir2_dist = IR_dist(LEFT_BACK);

  // if already aligned return
  if(abs(ir1_dist - ir2_dist) <= alignment_threshold) {
    return;
  }

  // update IR distances until aligned
  do {
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
    
    diff = ir1_dist - ir2_dist;
    power = Kp_align * diff;
    
    // send power
    left_front_motor.writeMicroseconds(1500 - power);
    left_rear_motor.writeMicroseconds(1500 - power);
    right_front_motor.writeMicroseconds(1500 - power);
    right_rear_motor.writeMicroseconds(1500 - power);
       
  } while(abs(diff) >= alignment_threshold);

  stop(); // stop motors
  return;
}

void align() { // uses long range IRs to align robot to wall - NO CONTROLLER
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.3;

  // read long range IRs
  ir1_dist = IR_dist(LEFT_FRONT);
  ir2_dist = IR_dist(LEFT_BACK);

  // if already aligned return
  if(abs(ir1_dist - ir2_dist) <= alignment_threshold) {
    return;
  }

  // determine direction of rotation for correction
  if(ir1_dist > ir2_dist) {
    ccw(70);
  }
  else {
    cw(70);
  }

  // update IR distances until aligned
  do {
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
       
  } while(abs(ir1_dist - ir2_dist) >= alignment_threshold);

  stop(); // stop motors
  return;
}


void align_back() { //uses mid range IRs to align to wall - NO CONTROLLER
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.25;

  // read long range IRs
  ir1_dist = IR_dist(BACK_LEFT);
  ir2_dist = IR_dist(BACK_RIGHT);

  // if already aligned return
  if(abs(ir1_dist - ir2_dist) <= alignment_threshold) {
    return;
  }

  // determine direction of rotation for correction
  if(ir1_dist > ir2_dist) {
    ccw(80);
  }
  else {
    cw(80);
  }

  // update IR distances until aligned
  do {
    ir1_dist = IR_dist(BACK_LEFT);
    ir2_dist = IR_dist(BACK_RIGHT);
    
  } while(abs(ir1_dist - ir2_dist) >= alignment_threshold);

  stop(); // stop motors
  return;
  
}

void driveXYZ(float x, float y, float z) { // Drives robot straight in x, y, and z // turning only occurs by itself i.e. no x and y
  DRIVING = true;

  if(x == 0) { // if x has been inputted 0 - don't care
    ignore_x = true;
  }
  
  if(y == 0) { // if y has been inputted 0 - don't care
    ignore_y = true;
  }

  if(z == 0) { // if y has been inputted 0 - don't care
    ignore_z = true;
  }

  reference[0] = x;
  reference[1] = y;
  reference[2] = z;

  while(DRIVING) {
    Controller(); // need to set DRIVING as false when stopped
  }

  stop(); // stop motors
  
  // reset flags
  //ultra_first_call = true;
  //pid_first_call = true;
  ignore_x = false;
  ignore_y = false;
  ignore_z = false;
}

void rotate(float angle) { // Turns robot to ensure alignment +ve = CW, -ve = CCW - USES P CONTROL
  float rotation = 0; 
  float starting_angle = 0;
  float power;
  
  currentAngle = 0;

  starting_angle = gyroAngle(); // get initial angle
  
  do {
    rotation = gyroAngle();
    power = (angle - rotation) * Kp_turn;

    // constrain
    if(power > 250) {
      power = 250;
    }
    else if(power < -250) {
      power = -250;
    }

    // send power
    left_front_motor.writeMicroseconds(1500 + power);
    left_rear_motor.writeMicroseconds(1500 + power);
    right_rear_motor.writeMicroseconds(1500 + power);
    right_front_motor.writeMicroseconds(1500 + power);
  } while(abs(rotation) < abs(angle)); // exit condition
  
  align(); // align with the wall
  
  return;
}
//----WRITTEN DRIVING FUNCTIONS----

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
  IR_Sensors(); // gets info from IRs including wall dist and drift correction factor
  correction = Kp_straight * IR_diff;

  if(!ignore_x) {
    if(reference[0] == 185 && IR_mid_dist < 30) { // switch to mid range IRs for x direction control when reversing 15cm away
      error[0] = IR_mid_dist - 15.0;
    }
    else {
      error[0] = reference[0] - Ultrasound();
    }
  }
  if(!ignore_y) { // if ignore y flag is false - update y error
    error[1] = reference[1] - IR_wall_dist;
  }
  if(!ignore_z) {
    error[2] = reference[2] - gyroAngle();
    correction = 0;
  }

  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();

  // exit condition
  if((abs(error[0]) + abs(error[1])) < 3.0 && (ignore_z)) { // XY exit condition
    DRIVING = false;
    return;
  }
  else if(abs(error[2]) < 0.1 && (!ignore_z)) { // turning exit condition
    DRIVING = false;
    currentAngle = 0;
    return;
  }

  delay(20);
}

void PID_Controller(){

  currentTime = millis();                
  elapsedTime = (currentTime - previousTime)/1000.0; 
 
  //if((currentTime - previousTime) >= pid_sample_time || (pid_first_call)) { // sampled at 20Hz
    for (int i = 0; i < 3; i++){
  
      Pterm[i] = Kp_r[i] * error[i];
      Iterm[i] += Ki_r[i] * error[i] * elapsedTime;
      Dterm[i] = Kd_r[i] * ((error[i] - lastError[i]) / elapsedTime);

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
    //pid_first_call = false;
    previousTime = currentTime; 
  //}
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
  left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0] - correction);
  left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1] - correction);
  right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2] - correction);
  right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3] - correction);
}

float gyroAngle() { // BASIC FOR TESTING - WILL LIKELY REPLACE WITH TURNING CONTROLLER
  gyroTime = millis();
  timeElapsed = gyroTime - prev_gyroTime;
  
  gyroRate = ((analogRead(gyroPin) - gyroMiddle) * gyroSupplyVoltage) / 1024.0; 
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second
  
  if(angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    angleChange = angularVelocity * (timeElapsed / 1000.0);
    currentAngle += angleChange;
    radiansAngle = currentAngle * (PI/180.0);
  }
  
  delay(5);

  prev_gyroTime = gyroTime;
  return radiansAngle;
}

float Ultrasound() {
  float Ultradistance;
  //ultra_time = millis() - prev_ultra_time;
  
  //if(ultra_time >= ultra_sampling_time || (ultra_first_call)) { //20Hz
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    Ultraduration = pulseIn(echoPin, HIGH);
    
    // Calculating the distance from robot centre
    Ultradistance = ultra_centre_offset + (Ultraduration * 0.034 / 2);

    //prev_ultra_time = ultra_time;

    //ultra_first_call = false;
  //}
  
  return Ultradistance;
}

void IR_Sensors(){
  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);

  IR_MID_1_DIST = IR_dist(BACK_LEFT);
  IR_MID_2_DIST = IR_dist(BACK_RIGHT);
  
  IR_wall_dist = long_centre_offset + (0.37 * IR_LONG_1_DIST + 0.63 * IR_LONG_2_DIST); // distance of centre of robot to wall
  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  //IR_Angle = atan((IR_diff/IR_Between_Dist));
  
  IR_mid_dist = mid_centre_offset + ((IR_MID_1_DIST + IR_MID_2_DIST) / 2);
  IR_mid_diff = IR_MID_1_DIST - IR_MID_2_DIST;
}
  
float IR_dist(IR code) { // find distances using calibration curve equations
  float est, dist;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      dist = (5780.3)/(pow(adc,1.027));
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
      
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      dist = (4382.9)/(pow(adc,0.984));
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

      break;
    case BACK_LEFT:
      adc = analogRead(IR_MID_1);
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
      
      break;
    case BACK_RIGHT:
      adc = analogRead(IR_MID_2);
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
