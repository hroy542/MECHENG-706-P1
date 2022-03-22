//----MotorSetup----
# include <Servo.h>   // include the library of servo motor control
// define the control pin of each motor
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

Servo left_front_motor;
Servo left_rear_motor;
Servo right_rear_motor;
Servo right_front_motor;
//----MotorSetup----

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

double reference_x = 15.0;
double reference_y = 0;
double reference_z = 0;
//----PIDValues----


//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;

int Ultradistance = 0;
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
double sensor_noise = 5;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----

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

void setup() {
pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
pinMode(echoPin, INPUT); // Sets the echoPin as an Input
Serial.begin(9600);

left_front_motor.attach(left_front);
left_rear_motor.attach(left_rear);
right_rear_motor.attach(right_rear);
right_front_motor.attach(right_front);
}


void loop() {

  StraightLineController(reference_x, reference_y, reference_z, Kp_x, Ki_x, Kp_y, Ki_y, Kp_z, Ki_z);  

} 

void StraightLineController(double reference_x, double reference_y, double reference_z, double Kp_x, double Ki_x, double Kp_y, double Ki_y, double Kp_z, double Ki_z){
  Ultradistance = ultrasonic_dist(); // Get distance from front of robot to wall

  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);

  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  correction = IR_diff * Kp_straight; // drift correction factor
  
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
  constrain(FLspeed_val, 0, 400);
  double BLspeed_val = WriteMicroseconds(BL_Ang_Vel/10);
  constrain(BLspeed_val, 0, 400);
  double BRspeed_val = WriteMicroseconds(BR_Ang_Vel/10);
  constrain(BRspeed_val, 0, 400);
  double FRspeed_val = WriteMicroseconds(FR_Ang_Vel/10);
  constrain(FRspeed_val, 0, 400);

  //Serial.println(FLspeed_val);
  
  left_front_motor.writeMicroseconds(1500 + FLspeed_val - correction);
  left_rear_motor.writeMicroseconds(1500 + BLspeed_val - correction);
  right_rear_motor.writeMicroseconds(1500 - BRspeed_val - correction);
  right_front_motor.writeMicroseconds(1500 - FRspeed_val - correction);
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

double WriteMicroseconds (double AngVel){
  int Microseconds = 430 - sqrt(160149 - (10000*AngVel));
  return Microseconds;
}

int ultrasonic_dist() {
  int dist;
  long Ultraduration;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  Ultraduration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  dist = 11.5 + (Ultraduration * 0.034 / 2); // calculates distance from centre of robot
  return dist;
}

double IR_dist(IR code) { // find distances using calibration curve equations
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
  }
      
  est = Kalman(dist, last_est);
  last_est = est;  
      
  // might need delay dunno
  delay(1);
      
  return est;
}

// Kalman Filter for IR sensors
double Kalman(double rawdata, double prev_est) {   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}
