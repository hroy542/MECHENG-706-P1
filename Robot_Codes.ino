
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

//Stage when running
enum STAGE {
  ORIENT,
  DRIVE,
  TURN,
  END,
};

static STAGE operation_stage = ORIENT;

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;

long Ultraduration;
int Ultradistance;
//----Ultrasound----

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

double IR_diff = 0;
double correction = 0;

// Back left mid range IR
const int IR_MID_1 = A6;
double IR_MID_1_DIST = 0;

// Back right mid range IR
const int IR_MID_2 = A7;
double IR_MID_2_DIST = 0;
//----IR----

//----IR Kalman Filter----
double last_est = 0;
double last_var = 999;
double process_noise = 1;
double sensor_noise = 10;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

//----Gyro----
#define PI 3.1415926535897932384626433832795
const int gyroPin = A8;           //define the pin that gyro is connected  
int T = 100;                        // T is the time of one loop, 0.1 sec  
int gyroADC = 0;           // read out value of sensor  
float gyroSupplyVoltage = 5;      // supply voltage for gyro 
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero  
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet  
float rotationThreshold = 1.5;      // because of gyro drifting, defining rotation angular velocity less  
                                                       // than this value will be ignored 
float gyroRate = 0;                      // read out value of sensor in voltage   
float currentAngle = 0;               // current angle calculated by angular velocity integral on  
byte serialRead = 0;

double radiansAngle = 0;
//----Gyro----

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

Servo left_front_motor;  // create servo object to control Vex Motor Controller 29
Servo left_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_rear_motor;  // create servo object to control Vex Motor Controller 29
Servo right_front_motor;  // create servo object to control Vex Motor Controller 29
//Servo turret_motor;

//----InverseKinematicValues----
double L = 7.620;
double l = 9.078;
double R_w = 2.54;
//----InverseKinematicValues----

//----PIDValues----
double Kp_x = 3.38;    double Ki_x = 0.154;
double Kp_y = 1.96;    double Ki_y = 0.205;
double Kp_z = 2.72;    double Ki_z = 0.343;

double Kp_straight = 20; // gain for keeping robot straight

double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

double reference_x = 10;
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
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Project_1_Code_2022");
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
    Analog_Range_A4();//What Does this do? - Hritom

  switch(operation_stage) {
    case ORIENT:
      orient();
      break;
    case DRIVE:
      drive();
      break;
    case TURN:
      turn();
      break;
    case END:
      stop();
      break;
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
  while (digitalRead(echoPin) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while (digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if (pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = 11.5 + (pulse_width / 58.0); // distance from centre of robot
  
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

void Analog_Range_A4()//Why do we need this
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A7));
}
#endif

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

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

void orient() { // Drives robot to TL or BR corner
  // PSEUDOCODE
  // Rotate slowly taking readings from ultrasonic sensor
  // Minimum reading taken - perpendicular to wall
  // Align and drive straight (driveX) ~20cm from wall
  // Turn 180 degrees (turn funciton - gyro) and measure distance to other wall with ultrasonic (determine which side)
  // If short side, reverse 5cm, then strafe left (driveY function) to starting position (15cm away)
  // If long side, rotate 90 degrees CCW, strafe left 5 cm, then reverse to starting position (15cm away)
  
  operation_stage = DRIVE;
  
}
  
void align() { // aligns robot perpendicular to wall
  
  
}

void drive() { // Drives robot straight in X direction (forward/backwards) using PI
  StraightLineController(reference_x, reference_y, reference_z, Kp_x, Ki_x, Kp_y, Ki_y, Kp_z, Ki_z);  
}

void turn(int angle) { // Turns robot to ensure alignment +ve = CW, -ve = CCW
  
}

void stop() // Stops robot
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);
}
  
void forward()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void reverse ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void ccw ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void cw ()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void strafe_left ()
{
  left_front_motor.writeMicroseconds(1500 - speed_val);
  left_rear_motor.writeMicroseconds(1500 + speed_val);
  right_rear_motor.writeMicroseconds(1500 + speed_val);
  right_front_motor.writeMicroseconds(1500 - speed_val);
}

void strafe_right ()
{
  left_front_motor.writeMicroseconds(1500 + speed_val);
  left_rear_motor.writeMicroseconds(1500 - speed_val);
  right_rear_motor.writeMicroseconds(1500 - speed_val);
  right_front_motor.writeMicroseconds(1500 + speed_val);
}

void StraightLineController(double reference_x, double reference_y, double reference_z, double Kp_x, double Ki_x, double Kp_y, double Ki_y, double Kp_z, double Ki_z){
  //----Ultrasound----
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  Ultraduration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  Ultradistance = 11.5 + (Ultraduration * 0.034 / 2); // from centre of robot
  //Serial.print("Distance: ");
  //Serial.println(Ultradistance);
  //----Ultrasound----
  
  double V_x = PID_Controller(reference_x, Ultradistance, Kp_x, Ki_x);
  double V_y = PID_Controller(reference_y, 0, Kp_y, Ki_y);
  double V_z = PID_Controller(reference_z, 0, Kp_z, Ki_z);

  //Serial.println(V_x);
  
  FL_Ang_Vel = FL_InverseKinematics(V_x, V_y,  V_z);
  FR_Ang_Vel = FR_InverseKinematics( V_x, V_y, V_z);
  BL_Ang_Vel = BL_InverseKinematics( V_x,  V_y,  V_z);
  BR_Ang_Vel = BR_InverseKinematics( V_x, V_y,  V_z); 

  //Serial.println(FL_Ang_Vel);

  //Serial.print(FL_Ang_Vel);
  
  double FLspeed_val = WriteMicroseconds(FL_Ang_Vel/10);
  double BLspeed_val = WriteMicroseconds(BL_Ang_Vel/10);
  double BRpeed_val = WriteMicroseconds(BR_Ang_Vel/10);
  double FRspeed_val = WriteMicroseconds(FR_Ang_Vel/10);

  //Serial.println(FLspeed_val);
  
  left_front_motor.writeMicroseconds(1500 + FLspeed_val);
  left_rear_motor.writeMicroseconds(1500 + BLspeed_val);
  right_rear_motor.writeMicroseconds(1500 - BRspeed_val);
  right_front_motor.writeMicroseconds(1500 - FRspeed_val);
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
   cumError += error * elapsedTime;                
   rateError = (error - lastError)/elapsedTime;   
 
   double out = (-1)*(Kp*error + Ki*cumError);                          
 
   lastError = error;                               
   previousTime = currentTime;                       
 
   return out; 
}
  
double IR_filtered(IR code) { // find distances using calibration curve equations
  double est, dist;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      //dist =
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      //dist = 
      break;
    case BACK_LEFT:
      adc = analogRead(IR_MID_1);
      //dist =
      break;
    case BACK_RIGHT:
      adc = analogRead(IR_MID_2);
      //dist = 
      break;
      
  est = Kalman(dist, last_est);
  last_est = est;  
      
  // might need delay dunno
      
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
