// GOING TO CORNER - START
// REQUIRES DECENTLY ACCURATE GYRO - DOESNT NEED TO BE PERFECT (CAN ALIGN PERPENDICULAR USING IRS) 

#include <Servo.h>  //Need for Servo pulse output

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
//#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

// XY driving states
enum DIRECTION {
  FORWARD,
  REVERSE,
  LEFT,
  RIGHT,
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

int Ultradistance;
const double ultra_centre_offset = 10.5;
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
double IR_LONG_1_DIST = 0;

// Left back long range IR
const int IR_LONG_2 = A5;
double IR_LONG_2_DIST = 0;

double IR_long_diff = 0;
double correction = 0;
double IR_long_dist = 0;
const double long_centre_offset = 4.5;

// Back left mid range IR
const int IR_MID_1 = A6;
double IR_MID_1_DIST = 0;

// Back right mid range IR
const int IR_MID_2 = A7;
double IR_MID_2_DIST = 0;

double IR_mid_diff = 0;
double IR_mid_dist = 0;
const double mid_centre_offset = 7.0;
//----IR----

//----IR Kalman Filter----
double last_est = 0;
double last_var = 999;
double process_noise = 1;
double sensor_noise = 25;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----Gyro----
const int gyroPin = A8;           //define the pin that gyro is connected  
int T = 100;                        // T is the time of one loop, 0.1 sec  
int gyroADC = 0;           // read out value of sensor  
float gyroSupplyVoltage = 5;      // supply voltage for gyro 
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero  
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet  
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity less  
float angularVelocity = 0;
                                                       // than this value will be ignored 
int timeElapsed = 0;
int prevTime = 0;
float gyroRate = 0;                      // read out value of sensor in voltage   
float angleChange = 0;
float currentAngle = 0;               // current angle calculated by angular velocity integral on  
byte serialRead = 0;

double radiansAngle = 0;
//----Gyro----

//----Go to Corner Variables----
bool perpendicular = false;
bool LONG = false;
bool SHORT = false;
bool DRIVING = false;
//----Go to Corner Variables----

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
Servo turret_motor;

//----InverseKinematics----
double FL_Ang_Vel = 0;
double FR_Ang_Vel = 0;
double BL_Ang_Vel = 0;
double BR_Ang_Vel = 0;

double FLspeed_val = 0;
double BLspeed_val = 0;
double BRspeed_val = 0;
double FRspeed_val = 0;
//----InverseKinematics----

//----InverseKinematicValues----
double L = 7.620;
double l = 9.078;
double R_w = 2.54;
//----InverseKinematicValues----

//----PIDValues----
double Kp_x = 3.38;    double Ki_x = 0.154;
double Kp_y = 1.96;    double Ki_y = 0.205;
double Kp_z = 2.72;    double Ki_z = 0.343;

double Kp_straight = 100; // gain for keeping robot straight

double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;
int intFactor = 1;

double reference_x = 0;
double reference_y = 0;
double reference_z = 0;
//----PIDValues----

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

  find_corner();

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
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
    
//  orient(); // initial orient of robot perpendicular to wall
//  
//  double IR_long_dist = 0.7 * IR_dist(LEFT_FRONT) + 0.3 * IR_dist(LEFT_BACK);
//
//  driveXY(20, IR_long_dist, FORWARD);
//
//  find_side(); // determines if on long or short side
//
//  if(LONG) {
//    corner_long();
//    LONG = false;
//  }
//  else if(SHORT) {
//    corner_short();
//    SHORT = false;
//  }

  align();
  disable_motors();

  // change FSM state - in full code
}

void orient() { // initial orienting of robot perpendicular to wall using IR
  double ir1_dist, ir2_dist, ratio;
  
  // while robot is not perpendicular to wall
  while(!perpendicular) {
    ccw(100); // rotate CCW
    
    // get distances for both IRs
    ir1_dist = IR_dist(LEFT_FRONT);
    ir2_dist = IR_dist(LEFT_BACK);
    
    // if both IRs are within 60cm
    if(ir1_dist < 60 && ir2_dist < 60) {
      
      // calculate ratio between IR distances
      if(ir1_dist < ir2_dist) {
        ratio = ir1_dist / ir2_dist;
      }
      else {
        ratio = ir2_dist / ir1_dist;
      }
      
      // if ratio is ~1 (i.e. nearly perpendicular) stop
      if(ratio >= 0.95) {
        stop();
        align(); // align to ensure fully perpendicular
        perpendicular = true; 
      }
    }
  }

  return;
}

void find_side() { // determine whether on short or long side of rectangle
  rotate(175); // rotate robot to see other wall
  int ultra = ultrasonic_dist();

  // if ultrasonic detects > 160 cm - robot on long side
  if(ultra > 160) {
    LONG = true;
    SHORT = false;
  }
  // if ultrasonic detects < 130 cm - robot on short side
  else if(ultra < 130) {
    SHORT = true;
    LONG = false;
  }

  return;
}

void corner_long() { // drives robot to corner if on long side
  int ultra_dist;
  
  rotate(-87); // rotate 90 degrees CCW
  
  ultra_dist = ultrasonic_dist(); // read distance from wall
  driveXY(ultra_dist, 15, LEFT); // strafe left until 15cm from wall
  
  driveXY(185, 15, REVERSE); // reverse until 15cm from wall (starting position)
  align();
  
  stop();
  LONG = false;
}

void corner_short() { // drives robot to corner if on short side
  driveXY(185, 0, REVERSE); // reverse until 15cm from wall
  align();
  
  driveXY(185, 15, LEFT); // strafe left into starting position
  align();
}
  
void align() { // aligns robot perpendicular to wall
  // basic - might need PID control to ensure accuracy
  double ir1_dist, ir2_dist;
  double alignment_threshold = 0.1;

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

void driveXY(double x, double y, DIRECTION dir) { // Drives robot straight in X or Y direction (forward/backwards) using PI control
  DRIVING = true;
  
  switch(dir) {
    case FORWARD:
      while(DRIVING) {
        StraightLineController(x, y, 0, FORWARD);
      }
      break;
    case REVERSE:
      while(DRIVING) {
        StraightLineController(x, y, 0, REVERSE);
      }
      break;
    case LEFT:
      while(DRIVING) {
        StraightLineController(x, y, 0, LEFT);
      }
      break;
    case RIGHT:
      while(DRIVING) {
        StraightLineController(x, y, 0, RIGHT);
      }
      break;
  } 
}

void rotate(int angle) { // Turns robot to ensure alignment +ve = CW, -ve = CCW - WILL LIKELY REPLACE WITH TURNING CONTROLLER
  double rotation = 0; 
  double starting_angle = 0;
  currentAngle = 0;

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
  
  //align(); // align with the wall
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


//----WRITTEN HELPER FUNCTIONS----
void StraightLineController(double reference_x, double reference_y, double reference_z, DIRECTION dir) {
  double V_x, V_y, V_z;
  
  Ultradistance = ultrasonic_dist(); // Get distance from front of robot to wall

  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);

  IR_long_dist = long_centre_offset + (0.3 * IR_LONG_1_DIST + 0.7 * IR_LONG_2_DIST); // distance of centre of robot to wall

  // if input for y is 0, don't take vary V_y
  if(reference_y == 0) {
    IR_long_dist = 0;
  }

  IR_long_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  correction = IR_long_diff * Kp_straight; // drift correction factor
  
  // Only use mid range IRs when reversing 15cm away from wall
  if(reference_x == 185) {
    IR_MID_1_DIST = IR_dist(BACK_LEFT);
    IR_MID_2_DIST = IR_dist(BACK_RIGHT);
    IR_mid_dist = (IR_MID_1_DIST + IR_MID_2_DIST) / 2;
      
    V_x = PID_Controller(15, IR_mid_dist, Kp_x, Ki_x);
  }
  else { // Else use ultrasonic for x driving
    V_x = PID_Controller(reference_x, Ultradistance, Kp_x, Ki_x);
  }
  V_y = PID_Controller(reference_y, IR_long_dist, Kp_y, Ki_y);
  V_z = PID_Controller(reference_z, 0, Kp_z, Ki_z);

  //Serial.println(V_x);
  
  FL_Ang_Vel = FL_InverseKinematics(V_x, V_y, V_z);
  FR_Ang_Vel = FR_InverseKinematics(V_x, V_y, V_z);
  BL_Ang_Vel = BL_InverseKinematics(V_x, V_y, V_z);
  BR_Ang_Vel = BR_InverseKinematics(V_x, V_y, V_z); 

  //Serial.println(FL_Ang_Vel);

  //Serial.print(FL_Ang_Vel);
  
  double FLspeed_val = WriteMicroseconds(FL_Ang_Vel/100);
  constrain(FLspeed_val, 0, 500);
  double BLspeed_val = WriteMicroseconds(BL_Ang_Vel/100);
  constrain(BLspeed_val, 0, 500);
  double BRspeed_val = WriteMicroseconds(BR_Ang_Vel/100);
  constrain(BRspeed_val, 0, 500);
  double FRspeed_val = WriteMicroseconds(FR_Ang_Vel/100);
  constrain(FRspeed_val, 0, 500);

  //Serial.println(FLspeed_val);
  switch(dir) {
    case FORWARD:
      left_front_motor.writeMicroseconds(1500 + FLspeed_val - correction);
      left_rear_motor.writeMicroseconds(1500 + BLspeed_val - correction);
      right_rear_motor.writeMicroseconds(1500 - BRspeed_val - correction);
      right_front_motor.writeMicroseconds(1500 - FRspeed_val - correction);
      break;
    case REVERSE:
      left_front_motor.writeMicroseconds(1500 - FLspeed_val - correction);
      left_rear_motor.writeMicroseconds(1500 - BLspeed_val - correction);
      right_rear_motor.writeMicroseconds(1500 + BRspeed_val - correction);
      right_front_motor.writeMicroseconds(1500 + FRspeed_val - correction);
      break;
    case LEFT:
      left_front_motor.writeMicroseconds(1500 - FLspeed_val - correction);
      left_rear_motor.writeMicroseconds(1500 + BLspeed_val - correction);
      right_rear_motor.writeMicroseconds(1500 + BRspeed_val - correction);
      right_front_motor.writeMicroseconds(1500 - FRspeed_val - correction);
      break;
    case RIGHT:
      left_front_motor.writeMicroseconds(1500 + FLspeed_val - correction);
      left_rear_motor.writeMicroseconds(1500 - BLspeed_val - correction);
      right_rear_motor.writeMicroseconds(1500 - BRspeed_val - correction);
      right_front_motor.writeMicroseconds(1500 + FRspeed_val - correction);
      break;
  }
}
  
double FL_InverseKinematics(double v_x, double v_y, double omega_z){
  double FL_Ang_Vel;
  FL_Ang_Vel = (v_x + v_y -(omega_z*(L+l)))/R_w;
  return FL_Ang_Vel;
}

double FR_InverseKinematics(double v_x, double v_y, double omega_z){
  double FR_Ang_Vel;
  FR_Ang_Vel = (v_x - v_y +(omega_z*(L+l)))/R_w;
  return FR_Ang_Vel;
}

double BL_InverseKinematics(double v_x, double v_y, double omega_z){
  double BL_Ang_Vel;
  BL_Ang_Vel = (v_x - v_y -(omega_z*(L+l)))/R_w;
  return BL_Ang_Vel;
}

double BR_InverseKinematics(double v_x, double v_y, double omega_z){
  double BR_Ang_Vel;
  BR_Ang_Vel = (v_x + v_y +(omega_z*(L+l)))/R_w;
  return BR_Ang_Vel;
}

double PID_Controller (double reference, double current, double Kp, double Ki){
   //Serial.println("PIDTest");
   unsigned long currentTime, previousTime;
   double elapsedTime, error, cumError, rateError, lastError;
   
   currentTime = millis();                
   elapsedTime = (double)(currentTime - previousTime);       
        
   error = reference - current;     
   //Serial.println(error); 
  
  // anti-windup
   if(lastError == error) { // i.e. stopped
     intFactor = 0;
     DRIVING = false;
   }
   else {
     intFactor = 1;
   }
  
   cumError += error * elapsedTime;                
   rateError = (error - lastError)/elapsedTime;   
 
   double out = (Kp*error + Ki*cumError*intFactor);                          
 
   lastError = error;                               
   previousTime = currentTime;                       
 
   return out; 
}

double WriteMicroseconds (double AngVel){
  int Microseconds = 430 - sqrt(160149 - (10000*AngVel));
  return Microseconds;
}

double gyroAngle() { // BASIC FOR TESTING - WILL LIKELY REPLACE WITH TURNING CONTROLLER
  timeElapsed = millis() - prevTime;
  prevTime = millis();
  
  gyroRate = ((analogRead(gyroPin) - 505) * gyroSupplyVoltage) / 1023.0; 
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second
  angleChange = angularVelocity * (timeElapsed / 1000.0);
  currentAngle += angleChange;
  Serial.println(currentAngle);

  delay(20);
  
  return currentAngle;
}

int ultrasonic_dist() {
  int dist; // cm from centre of robot to wall
  long Ultraduration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  Ultraduration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  dist = ultra_centre_offset + (Ultraduration * 0.034 / 2); // calculates distance from centre of robot
  return dist;
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
      
  delay(10);
      
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
//----WRITTEN HELPER FUNCTIONS----
