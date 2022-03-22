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

double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

double reference_x = 0;
double reference_y = 0;
double reference_z = PI/2;
//----PIDValues----


//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;

long Ultraduration;
int Ultradistance;
//----Ultrasound----

//----Kalman----
double last_est = 0;
double last_var = 999;
double process_noise = 1;
double sensor_noise = 15; 
//----Kalman----

//----Gyro----
#define PI 3.1415926535897932384626433832795
const int sensorPin = A7;           //define the pin that gyro is connected  
int T = 100;                        // T is the time of one loop, 0.1 sec  
int sensorValue = 0;           // read out value of sensor  
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


//----Gyro----
int i; 
float sum = 0;     
pinMode(sensorPin,INPUT);   
 
Serial.println("please keep the sensor still for calibration"); 
Serial.println("get the gyro zero voltage"); 
for (i=0;i<100;i++)    // read 100 values of voltage when gyro is at still, to calculate the zero-drift.  
{  
  sensorValue = analogRead(sensorPin); 
  sum += sensorValue; 
  delay(5); 
  } 
gyroZeroVoltage = sum/100;    // average the sum as the zero drifting  
} 
//----Gyro----


void loop() {
//delay(2000);
TurningController(reference_x, reference_y, reference_z, Kp_x, Ki_x, Kp_y, Ki_y, Kp_z, Ki_z);     
} 


void TurningController(double reference_x, double reference_y, double reference_z, double Kp_x, double Ki_x, double Kp_y, double Ki_y, double Kp_z, double Ki_z){
  //----Ultrasound----
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  Ultraduration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  Ultradistance = 11.5 + Ultraduration * 0.034 / 2; // distance from centre of robot
  //Serial.print("Distance: ");
  //Serial.println(Ultradistance);
  //----Ultrasound----

  //----Gyro----
  // convert the 0-1023 signal to 0-5v 
  gyroRate = (analogRead(sensorPin)*gyroSupplyVoltage)/1023;    
   
  // find the voltage offset the value of voltage when gyro is zero (still) 
  gyroRate -= (gyroZeroVoltage/1023*gyroSupplyVoltage);    
   
  // read out voltage divided the gyro sensitivity to calculate the angular velocity  
  float angularVelocity = gyroRate/ gyroSensitivity; // from Data Sheet, gyroSensitivity is 0.007 V/dps 
   
  // if the angular velocity is less than the threshold, ignore it 
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) 
  { 
    // we are running a loop in T (of T/1000 second).  
    float angleChange = angularVelocity/(1000/T); 
    currentAngle += angleChange;  
  } 
   
  // keep the angle between 0-360 
  if (currentAngle < 0) 
    {currentAngle += 360;} 
  else if (currentAngle > 359) 
    {currentAngle -= 360;} 
  
  //Serial.println(currentAngle);
  //delay(500);
  radiansAngle = currentAngle*(PI/180); 
  //Serial.print(angularVelocity); 
  //Serial.print(" "); 
  //Serial.println(currentAngle); 
   
  // control the time per loop 
  delay (T); 
  
  //----Controller----  
  double V_x = PID_Controller(reference_x, 0, Kp_x, Ki_x);
  double V_y = PID_Controller(reference_y, 0, Kp_y, Ki_y);
  double V_z = PID_Controller(reference_z, radiansAngle, Kp_z, Ki_z);

  //Serial.println(V_z);
  
  FL_Ang_Vel = FL_InverseKinematics(V_x, V_y,  V_z);
  FR_Ang_Vel = FR_InverseKinematics( V_x, V_y, V_z);
  BL_Ang_Vel = BL_InverseKinematics( V_x,  V_y,  V_z);
  BR_Ang_Vel = BR_InverseKinematics( V_x, V_y,  V_z); 

  //Serial.println(FL_Ang_Vel);
  
  double FLspeed_val = WriteMicroseconds(FL_Ang_Vel/1000);
  constrain(FLspeed_val, 0, 500);
  double BLspeed_val = WriteMicroseconds(BL_Ang_Vel/1000);
  constrain(BLspeed_val, 0, 500);
  double BRspeed_val = WriteMicroseconds(BR_Ang_Vel/1000);
  constrain(BRspeed_val, 0, 500);
  double FRspeed_val = WriteMicroseconds(FR_Ang_Vel/1000);
  constrain(FRspeed_val, 0, 500);

  //Serial.println(FLspeed_val);
  Serial.println(FRspeed_val);

  // CCW Rotation
  left_front_motor.writeMicroseconds(1500 - FLspeed_val);
  left_rear_motor.writeMicroseconds(1500 - BLspeed_val);
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
        
   //error = reference - current;  
   error = PI/2;
   //Serial.println(error);
   //delay(1000);   
   //Serial.println(error);                          
   cumError += error * elapsedTime;                
   rateError = (error - lastError)/elapsedTime;   
 
   double out = (-1)*(Kp*error + Ki*cumError);

   //Serial.println(out);   

   //Serial.print(out);
 
   lastError = error;                               
   previousTime = currentTime;                       
 
   return out; 
}

double WriteMicroseconds (double AngVel){
  double Microseconds = 265 - sqrt((88035/2) - (25000*AngVel));
  return Microseconds;
}

double Kalman(double RawData, double prev_est) { // Kalman filter for gyro readings
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}
//----Controller----
