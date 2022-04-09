// CONTROLLER FOR TESTING DRIVING - TUNE PID AND SAMPLING DELAYS // 

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
float max_velocity[3] = {25,20,1.5}; // cm/s - rad/s
float ang_vel[4] = {0,0,0,0};

float motor_speed_Value[4] = {0,0,0,0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {20,20,0};
float currentTime, previousTime, elapsedTime;
float error[3] = {0,0,0};
float lastError[3] = {0,0,0};
float rateError[3] = {0,0,0};

//const int pid_sample_time = 0; // 40ms sampling period - 25Hz
bool pid_first_call = true;

//StraightLine
float Kp_r[3] = {2,2.5,1.65};
float Ki_r[3] = {0.018,0.02,0.05};
float Kd_r[3] = {0,0,0};

float Pterm[3], Iterm[3], Dterm[3];

float Kp_straight = 90;
//----PIDValues----

//----Gyro----
const int gyroPin = A8;           //define the pin that gyro is connected  
int T = 100;                        // T is the time of one loop, 0.1 sec  
int gyroADC = 0;           // read out value of sensor  
float gyroSupplyVoltage = 5;      // supply voltage for gyro 
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero  
float gyroSensitivity = 0.007;      // gyro sensitivity unit is (mv/degree/second) get from datasheet  
float rotationThreshold = 3;      // because of gyro drifting, defining rotation angular velocity less  
float angularVelocity = 0;
                                                       // than this value will be ignored 
int gyroTime = 0;
int prev_gyroTime = 0;
int timeElapsed = 0;
float gyroRate = 0;                      // read out value of sensor in voltage   
float angleChange = 0;
float currentAngle = 0;               // current angle calculated by angular velocity integral on  
float radiansAngle = 0;
//----Gyro----

//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;
float Ultraduration;
float Ultradistance = 0;
const float ultra_centre_offset = 11.0;

//const int ultra_sampling_time = 0; //40ms sampling time
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
//MA Filter -------------


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
  Ultrasound();
  IR_Sensors();
  if(reference[0] == 185 && IR_mid_dist < 30) {
    error[0] = IR_mid_dist - 15.0;
  }
  else {
    error[0] = reference[0] - Ultradistance;
  }
  error[1] = reference[1] - IR_wall_dist;
  error[2] = reference[2] - radiansAngle;
  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  set_motors();
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

void Ultrasound(){
  //ultra_time = millis();
  
  //if((ultra_time - prev_ultra_time) >= ultra_sampling_time || (ultra_first_call)) { //20Hz
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    Ultraduration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    Ultradistance = ultra_centre_offset + (Ultraduration * 0.034 / 2);

   // prev_ultra_time = ultra_time;
    ultra_first_call = false;
 // }
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
  
  correction = Kp_straight * IR_diff;
}

void gyro() { // could be tuned better
  gyroTime = millis();

  timeElapsed = gyroTime - prev_gyroTime;
  
  gyroRate = ((analogRead(gyroPin) - 505.0) * gyroSupplyVoltage) / 1024.0; 
  angularVelocity = gyroRate / gyroSensitivity; // angular velocity in degrees/second

  if(angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold) {
    angleChange = angularVelocity * (timeElapsed / 1000.0);
    currentAngle += angleChange;
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
