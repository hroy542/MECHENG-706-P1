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
float L = 007.620;
float l = 009.078;
float R_w = 002.54;

float velocity[3] = {0,0,0};
float max_velocity[3] = {25,20,15}; // cm/s
float ang_vel[4] = {0,0,0,0};

float motor_speed_Value[4] = {0,0,0,0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {20,20,PI};
float currentTime, previousTime, elapsedTime;
float max_velocities[3] = {500, 500, 600};
float error[3] = {0,0,0};
float lastError[3] = {0,0,0};
float rateError[3] = {0,0,0};

const int pid_sample_time = 50; // 50ms sampling period - 20Hz
int pid_time = 0;
int prev_pid_time = 0;
bool pid_first_call = true;

//StraightLine
float Kp_r[3] = {1.5,6,23};
float Ki_r[3] = {0.03,0.1,0.5};
float Kd_r[3] = {0,0,0};

float Pterm = 0;
float Iterm = 0;
float Dterm = 0;

float Kp_straight = 200;

//Turning
float Kp_t[3] = {0,0,3};
float Ki_t[3] = {0,0,0.05};
float Kd_t[3] = {0,0,0.5};
//----PIDValues----

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

float radiansAngle = 0;
const float rotation_scale_factor = 0.93; // rotation overshoot correction
//----Gyro----

//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;
float Ultraduration;
float Ultradistance = 0;
const float ultra_centre_offset = 10.75;

const int ultra_sampling_time = 50; //50ms sampling time as recommended in data sheet
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
float last_est = 0;
float last_var = 999;
float process_noise = 1;
float sensor_noise = 25;    // Change the value of sensor noise to get different KF performance
//----IR Kalman Filter----


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
  Controller();
}

void Controller(){
  gyro();
  //Ultrasound();
  //IR_Sensors();
  //error[0] = 0;
  error[0] = reference[0] - Ultradistance;
  error[1] = 0;//reference[1] - IR_wall_dist;
  error[2] = 0;//reference[2] - radiansAngle;
  //Serial.println(error[2]);
  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();
  int out = left_front_motor.readMicroseconds();
  Serial.println(out);
}

void PID_Controller(){

  currentTime = millis();                
  elapsedTime = (currentTime - previousTime)/1000.0; 
  
  pid_time = currentTime - prev_pid_time;

  if(pid_time >= pid_sample_time || (pid_first_call)) { // sampled at 20Hz
    for (int i = 0; i < 3; i++){
  
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
  
      velocity[i] = (Pterm)  +(Iterm) + (Dterm);
      
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

void set_motors() {
  //Serial.println(motor_speed_Value[0]);
  left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0] - correction);
  left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1] - correction);
  right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2] - correction);
  right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3] - correction);
}

void set_motor_speed(){
  for (int i = 0; i < 4; i++){
    motor_speed_Value[i] = ang_vel[i] * 32; // scale angular velocity to motor speed value - COULD BE TUNED BETTER
  }
}

void Ultrasound(){
  ultra_time = millis() - prev_ultra_time;
  
  if(ultra_time >= ultra_sampling_time || (ultra_first_call)) { //16.67Hz
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

void gyro() { // could be tuned better
  timeElapsed = millis() - prevTime;
  prevTime = millis();
  
  gyroRate = ((analogRead(gyroPin) - 505.0) * gyroSupplyVoltage) / 1023.0; 
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second
  angleChange = angularVelocity * (timeElapsed / 1000.0);
  currentAngle += angleChange;
  radiansAngle = currentAngle * (PI/180);
  
  delay(10);
}

double IR_dist(IR code) { // find distances using calibration curve equations
  double est, dist;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      //Serial.println(adc);
      dist = (6245.5)/(pow(adc,1.042));
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      dist = (2730.4)/(pow(adc,0.901));
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
double Kalman(double rawdata, double prev_est) { 
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}
