// GOING TO CORNER - START
// REQUIRES DECENTLY ACCURATE GYRO - DOESNT NEED TO BE PERFECT (CAN ALIGN parallel USING IRS) 

#include <Servo.h>  //Need for Servo pulse output
#include <SoftwareSerial.h>

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

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
const float ultra_centre_offset = 10.5;
const int ultra_sampling_time = 40; //40ms sampling time
int ultra_time;
int prev_ultra_time;
bool ultra_first_call = true;
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
float process_noise = 1;
float sensor_noise = 25;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----Gyro----
const int gyroPin = A8;           //define the pin that gyro is connected  
int gyroADC = 0;           // read out value of sensor  
float gyroSupplyVoltage = 5;      // supply voltage for gyro 
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet  
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity less  
float angularVelocity = 0;
                                                       // than this value will be ignored 
int timeElapsed = 0;
int prevTime = 0;
float gyroRate = 0;                      // read out value of sensor in voltage   
float angleChange = 0;
float currentAngle = 0;               // current angle calculated by angular velocity integral on  

float radiansAngle = 0;
//----Gyro----

//----Find Corner Flags----
bool parallel = false;
bool LONG = false;
bool SHORT = false;
//----Find Corner Flags----

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
Servo turret_motor;

//----InverseKinematicValues----
float L = 007.620;
float l = 009.078;
float R_w = 002.54;

float velocity[3] = {0,0,0};
float max_velocity[3] = {25,25,0}; // cm/s
float ang_vel[4] = {0,0,0,0};

float motor_speed_Value[4] = {0,0,0,0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {0,0,0};
float currentTime, previousTime, elapsedTime;
float max_velocities[3] = {500, 500, 600};
float error[3] = {0,0,0};
float lastError[3] = {0,0,0};
float rateError[3] = {0,0,0};

float Pterm, Iterm, Dterm;

//StraightLine
float Kp_r[3] = {1.3,3,1.65};
float Ki_r[3] = {0.01,0.3,0.05};
float Kd_r[3] = {0,0,0};

float Kp_straight = 80;

const int pid_sample_time = 40; // 40ms sampling period - 20Hz
bool pid_first_call = true;
//----PIDValues----

//----Localisation----
const int local_sample_time = 250; //4Hz sampling frequency for coordinates
int local_time = 0;
int prev_local_time = 0;
//----Localisation----

int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  delay(1000); //settling time but no really needed

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

  static unsigned long previous_millis;

  read_serial_command();
  fast_flash_double_LED_builtin();

  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
    previous_millis = millis();

    SerialCom->println("RUNNING---------");
    speed_change_smooth();
    Analog_Range_A4();
  }

#ifndef NO_READ_GYRO
    GYRO_reading();
#endif

#ifndef NO_HC-SR04
    HC_SR04_range();
#endif

#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif

  static RUN_STATE running_state = FIND_CORNER;

  //FSM
  switch (running_state) {
    case FIND_CORNER:
      running_state = find_corner();
      break;
    //----DRIVING COMPONENT----
    case FORWARD: 
      running_state =  forward();
      break;
    case REVERSE:
      running_state =  reverse();
      break;
    case STRAFE:
      running_state = strafe();
      break;
    case TURN:
      running_state =  turn();
      break;
    case END:
      running_state =  complete();
      break;
    //----DRIVING COMPONENT----
  };

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print message every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
  speed_change = 0;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(echoPin) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(echoPin) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = 10.5 + pulse_width / 58.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A8:");
  SerialCom->println(analogRead(gyroPin));
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        //forward (100);
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        //reverse (100);
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        //strafe_left(100);
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        //strafe_right(100);
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw(100);
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw(100);
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }

  }

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
    
  orient(); // initial orient of robot parallel to wall

  delay(500);
  
  float wall_dist = 0.63 * IR_dist(LEFT_FRONT) + 0.37 * IR_dist(LEFT_BACK);

  driveXY(20, wall_dist);

  find_side(); // determines if on long or short side

  if(LONG) {
    corner_long();
    LONG = false;
  }
  else if(SHORT) {
    corner_short();
    SHORT = false;
  }

  return FORWARD;
}

RUN_STATE forward() {
  forward_flag = true;

  if(turned) {
    driveXY(15, 45 - ((switch_back_count - 5) * 10)); 
  }
  else {
    driveXY(15, 15 + (switch_back_count * 10)); 
  }

  if(switch_back_count == 4) {
    driveXY(20, 15 + (switch_back_count * 10)); // reverse to allow for turning
    return TURN;
  }
  else {
    return STRAFE;
  }
}

RUN_STATE reverse() {
  forward_flag = false;

  if(turned) {
    driveXY(185, 45 - ((switch_back_count - 5) * 10)); // IF ULTRASONIC NOT ACCURATE ENOUGH - COULD USE MID RANGE IRS TO KEEP WALL DISTANCE CONSTANT
  }
  else {
    driveXY(185, 15 + (switch_back_count * 10)); 
  }

  if(switch_back_count == 8) {
    return END;
  }
  else {
    return STRAFE;
  }
}

RUN_STATE strafe() {
  switch_back_count++;

  if(turned) {
    if(forward) {
      driveXY(15, 45 - ((switch_back_count - 5) * 10));
      return REVERSE;
    } 
    else {
      driveXY(185, 45 - ((switch_back_count - 5) * 10));
      return FORWARD;
    }
  }
  else {
    if(forward) {
      driveXY(15, 15 + (switch_back_count * 10));
      return REVERSE;
    } 
    else {
      driveXY(185, 15 + (switch_back_count * 10));
      return FORWARD;
    }
  }
}

RUN_STATE turn() {
  switch_back_count++; // reset after turn
  turned = true;
  
  rotate(90);
  driveXY(45, 20);
  rotate(90); 

  return FORWARD;
}

RUN_STATE complete() {
  stop();
}

void orient() { // initial orienting of robot parallel to wall using IR
  float ir1_dist, ir2_dist, ratio;
  
  // while robot is not parallel to wall
  while(!parallel) {
    ccw(100); // rotate CCW
    
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
        align(); // align to ensure fully parallel
        parallel = true; 
      }
    }
  }

  return;
}

void find_side() { // determine whether on short or long side of rectangle
  float ultra_dist;
  
  rotate(180); // rotate robot to see other wall

  ultra_dist = Ultrasound();
  ultra_first_call = true;

  // if ultrasonic detects > 160 cm - robot on long side
  if(ultra_dist > 160) {
    LONG = true;
    SHORT = false;
  }
  // if ultrasonic detects < 130 cm - robot on short side
  else if(ultra_dist < 130) {
    SHORT = true;
    LONG = false;
  }

  return;
}

void corner_long() { // drives robot to corner if on long side
  float ultra_dist;
  
  rotate(-90); // rotate 90 degrees CCW

  ultra_dist = Ultrasound(); // read distance from wall
  ultra_first_call = true;
  
  driveXY(ultra_dist, 15); // strafe left until 15cm from wall
  
  driveXY(185, 15); // reverse until 15cm from wall (starting position)
  align();
  
  stop();
  LONG = false;
}

void corner_short() { // drives robot to corner if on short side
  driveXY(185, 0); // reverse until 15cm from wall
  align();
  
  driveXY(185, 15); // strafe left into starting position
  align();
}
  
void align() { // uses long range IRs to align robot to wall
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.1;

  // read long range IRs
  ir1_dist = IR_dist(LEFT_FRONT);
  ir2_dist = IR_dist(LEFT_BACK);

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
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
       
  } while(abs(ir1_dist - ir2_dist) >= alignment_threshold);

  stop(); // stop motors
  return;
}

void align_back() { //uses mid range IRs to align to wall
  float ir1_dist, ir2_dist;
  float alignment_threshold = 0.1;

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

void driveXY(float x, float y) { // Drives robot straight in X or Y direction (forward/backwards) using PI control
  DRIVING = true;

  reference[0] = x;
  reference[1] = y;

  while(DRIVING) {
    Controller(); // need to set DRIVING as false when stopped
  }
  
  // reset flags
  ultra_first_call = true;
  pid_first_call = true;
}

void rotate(int angle) { // Turns robot to ensure alignment +ve = CW, -ve = CCW - WILL LIKELY REPLACE WITH TURNING CONTROLLER
  double rotation = 0; 
  double starting_angle = 0;
  currentAngle = 0;
  
  angle = angle * rotation_scale_factor;

  starting_angle = gyroAngle(); // get initial angle

  // determine direction of rotation from input
  if(angle > 0) {
    cw(150);
  }
  else {
    ccw(150);
  }

  // continue running motor until given angle reached
  while(abs(rotation) < abs(angle)) {
    rotation = gyroAngle() - starting_angle;  
  }
  
  align(); // align with the wall
  stop(); // stop robot
  
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

//----CONTROL----
void Controller(){
  float Ultradistance = Ultrasound();
  IR_Sensors();
  //error[0] = 0;
  error[0] = reference[0] - Ultradistance;
  error[1] = reference[1] - IR_wall_dist;
  error[2] = 0;
  //Serial.println(error[2]);
  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();

  if(abs(error[0] + error[1]) < 2) {
    DRIVING = false;
  }
}

void PID_Controller(){

  currentTime = millis();                
  elapsedTime = (currentTime - previousTime)/1000.0; 
  
  pid_time = currentTime - prev_pid_time;

  if(pid_time >= pid_sample_time || (pid_first_call)) { // sampled at 20Hz
    
    for (int i = 0; i < 3; i++) {
      Pterm = Kp_r[i] * error[i];
      Iterm += Ki_r[i] * error[i] * elapsedTime;
      Dterm = Kd_r[i] * ((error[i]-lastError[i])/elapsedTime);
  
      // anti wind-up
      if(abs(Iterm) > max_velocity[i]) {
        if(Iterm < 0) {
          Iterm = -1 * max_velocity[i];
        }
        else {
          Iterm = max_velocity[i];
        }
      }

//      // anti wind-up - Iterm and Pterm (More powerful anti windup - constrains total control effort to always be <= max velocity)
//      if(abs(Pterm[i]) > max_velocity[i]) {
//        
//        // constrains Iterm such that Pterm + Iterm <= maximum velocity
//        if(Pterm[i] < 0) {
//          Pterm[i] = (-1 * max_velocity[i]);
//        }
//        else {
//          Pterm[i] = max_velocity[i];
//        }
//      }
//      if(abs(Iterm[i] + Pterm[i]) > max_velocity[i]) {
//        
//        // constrains Iterm such that Pterm + Iterm <= maximum velocity
//        if((Iterm[i] + Pterm[i]) < 0) {
//          Iterm[i] = (-1 * max_velocity[i]) - Pterm[i];
//        }
//        else {
//          Iterm[i] = max_velocity[i] - Pterm[i];
//        }
//      }
  
      velocity[i] = Pterm + Iterm + Dterm;
      
      // constrain to max velocity
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
    
    pid_first_call = false;
    previousTime = currentTime; 
    prev_pid_time = pid_time; 
  }
}


void inverse_kinematics(){
  ang_vel[0] = (-velocity[0] - velocity[1] + ((L+l)*velocity[2])) / R_w; // left front
  ang_vel[1] = (-velocity[0] + velocity[1] + ((L+l)*velocity[2])) / R_w; // left rear
  ang_vel[2] = (velocity[0] + velocity[1] + ((L+l)*velocity[2])) / R_w; // right rear
  ang_vel[3] = (velocity[0] - velocity[1] + ((L+l)*velocity[2])) / R_w; // right front
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
  timeElapsed = millis() - prevTime;
  prevTime = millis();
  
  gyroRate = ((analogRead(gyroPin) - 505) * gyroSupplyVoltage) / 1023.0; 
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second
  angleChange = angularVelocity * (timeElapsed / 1000.0);
  currentAngle += angleChange;
  
  delay(5);
  
  return currentAngle;
}

float Ultrasound() {
  float Ultradistance;
  ultra_time = millis() - prev_ultra_time;
  
  if(ultra_time >= ultra_sampling_time || (ultra_first_call)) { //20Hz
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    Ultraduration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    Ultradistance = ultra_centre_offset + (Ultraduration * 0.034 / 2);

    prev_ultra_time = ultra_time;

    ultra_first_call = false;
  }
  
  return Ultradistance;
}

void IR_Sensors(){
  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);
  //Serial.println(IR_LONG_2_DIST);
  
  IR_wall_dist = long_centre_offset + (0.37 * IR_LONG_1_DIST + 0.63 * IR_LONG_2_DIST); // distance of centre of robot to wall
  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  //Serial.println(IR_diff);
  IR_Angle = atan((IR_diff/IR_Between_Dist));
  correction = Kp_straight * IR_diff;
  //Serial.println(correction);
  //Serial.println(IR_Angle);
  //IR_Angle = IR_Angle*(3.1415/180);
  //Serial.println(IR_Angle);
}
  
double IR_dist(IR code) { // find distances using calibration curve equations
  double est, dist;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      dist = pow(adc, -1.042) * 6245.5;
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      dist = pow(adc, -0.901) * 2730.4;
      break;
    case BACK_LEFT:
      adc = analogRead(IR_MID_1);
      //dist =
      break;
    case BACK_RIGHT:
      adc = analogRead(IR_MID_2);
      //dist = 
      break;
  }
      
  est = Kalman(dist, last_est);
  last_est = est;  
      
  delay(1);
      
  return est;
}

// Kalman Filter for IR sensors
double Kalman(double rawdata, double prev_est){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}

//---------------Functions for Bluetooth Comms-----------------------------------
void serialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  //add localisation output here:
//  BluetoothSerial.print(Value1, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.print(Value2, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.println(Value3, DEC);
}

void serialOutputPlotter(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  //add localisation output here:
//  BluetoothSerial.print(Value1, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.print(Value2, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.println(Value3, DEC);
}

void bluetoothSerialOutputMonitor(int32_t Value1, int32_t Value2, int32_t Value3)
{
  String Delimiter = ", ";
  //add localisation output here:
//  BluetoothSerial.print(Value1, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.print(Value2, DEC);
//  BluetoothSerial.print(Delimiter);
//  BluetoothSerial.println(Value3, DEC);
}

void serialOutput(int32_t Value1, int32_t Value2, int32_t Value3)
{
  if (OUTPUTMONITOR)
  {
    //add localisation output here:
    //serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTPLOTTER)
  {
    //add localisation output here:
    //serialOutputMonitor(Value1, Value2, Value3);
  }

  if (OUTPUTBLUETOOTHMONITOR)
  {
    //add localisation output here:
    //serialOutputMonitor(Value1, Value2, Value3);
  }
}
//----WRITTEN HELPER FUNCTIONS----
