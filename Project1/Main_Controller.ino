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
float max_velocity[3] = {8,1,6};
float ang_vel[4] = {0,0,0,0};

float motor_speed_Value[4] = {0,0,0,0};
//----InverseKinematicValues----

//----PIDValues----
float reference[3] = {15,15,0};
float currentTime, previousTime, elapsedTime;
float max_velocities[3] = {500, 500, 600};
float error[3] = {0,0,0};
float cumError[3] = {0,0,0};
float lastError[3] = {0,0,0};
float rateError[3] = {0,0,0};

//StraightLine
float Kp_r[3] = {7,275,0.8};
float Ki_r[3] = {0.1,0.2,0.2};
float Kd_r[3] = {0,2,0.5};

//Turning
float Kp_t[3] = {0,0,3};
float Ki_t[3] = {0,0,0.05};
float Kd_t[3] = {0,0,0.5};

float Kp_straight = 300;
//----PIDValues----


//----Ultrasound----
const int trigPin = 34;
const int echoPin = 35;
float Ultraduration;
float Ultradistance;
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
float IR_Between_Dist = 12;


// Back left mid range IR
const int IR_MID_1 = A6;
double IR_MID_1_DIST = 0;

// Back right mid range IR
const int IR_MID_2 = A7;
float IR_MID_2_DIST = 0;
//----IR----

//MA Filter -------------
#define WINDOW_SIZE 13
int index_LFIR = 0;
double value_LFIR = 0;
double sum_LFIR = 0;
int index_LBIR = 0;
double value_LBIR = 0;
double sum_LBIR = 0;
double LEFT_FRONT_READING[WINDOW_SIZE];
double LEFT_BACK_READING[WINDOW_SIZE];
double averaged = 0;
//MA Filter -------------

int flag = 0;


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

  if(flag == 0){
  Ultrasound();
  IR_Sensors();
  //error[0] = 0;
  error[0] = (reference[0] - Ultradistance);
  error[1] = reference[1]-IR_wall_dist;
  //error[1] = 0;
  //error[2] = reference[2]-IR_Angle;
  //Serial.println(error[2]);
  error[2] = 0;
  PID_Controller();
  inverse_kinematics();
  set_motor_speed();
  //Serial.println(motor_speed_Value[1]);
  set_motors();
  StraightLineWithin();
  }
  
  
}


void StraightLineController(){
}

void PID_Controller(){
   
   currentTime = millis();                
   elapsedTime = (float)(currentTime - previousTime)/1000;  

   for (int i = 0; i < 3; i++){

    cumError[i] += error[i]*elapsedTime;
    rateError[i] = (error[i]-lastError[i])/elapsedTime;

    velocity[i] = (Kp_r[i] * error[i])+(Ki_r[i]*cumError[i])+(Kd_r[i]*rateError[i]);

        
    //if ((abs(velocity[i])) > max_velocity[i]){
    //  velocity[i] = ((abs(velocity[i]))/velocity[i] * max_velocity[i]);                         
    //}
 
   lastError[i] = error[i];                               
   previousTime = currentTime;                       
  }
}

void inverse_kinematics(){
  ang_vel[0] = R_w * (-velocity[0] + velocity[1] + ((L+l)*velocity[2]));
  ang_vel[1] = R_w * (-velocity[0] - velocity[1] + ((L+l)*velocity[2]));
  ang_vel[2] = R_w * (velocity[0] - velocity[1] + ((L+l)*velocity[2]));
  ang_vel[3] = R_w * (velocity[0] + velocity[1] + ((L+l)*velocity[2]));
}

void set_motors() {
  left_front_motor.writeMicroseconds(1500 + motor_speed_Value[0]-correction);
  left_rear_motor.writeMicroseconds(1500 + motor_speed_Value[1]-correction);
  right_rear_motor.writeMicroseconds(1500 + motor_speed_Value[2]-correction);
  right_front_motor.writeMicroseconds(1500 + motor_speed_Value[3]-correction);
}

void set_motor_speed(){
  for (int i = 0; i < 4; i++){
    motor_speed_Value[i] = ang_vel[i];

    if ((abs(motor_speed_Value[i])) > 500){
      motor_speed_Value[i] = ((abs(motor_speed_Value[i]))/motor_speed_Value[i])*500;
    }
  }
}

void Ultrasound(){
  //----Ultrasound----
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  Ultraduration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  Ultradistance = Ultraduration * 0.034 / 2;
  //Serial.println(Ultradistance);
//----Ultrasound----
}

void IR_Sensors(){
  // Get distances from left side of robot to wall
  IR_LONG_1_DIST = IR_dist(LEFT_FRONT);
  IR_LONG_2_DIST = IR_dist(LEFT_BACK);
  //Serial.println(IR_LONG_2_DIST);
  
  IR_wall_dist = 0.3 * IR_LONG_1_DIST + 0.7 * IR_LONG_2_DIST; // distance of centre of robot to wall
  IR_diff = IR_LONG_1_DIST - IR_LONG_2_DIST; // Difference between long range IRs
  //Serial.println(IR_diff);
  IR_Angle = atan((IR_diff/IR_Between_Dist));
  correction = IR_diff * Kp_straight;
  //Serial.println(IR_Angle);
  //IR_Angle = IR_Angle*(3.1415/180);
  //Serial.println(IR_Angle);
}


double IR_dist(IR code) { // find distances using calibration curve equations
  double est, dist, temp;
  int adc;
  
  switch(code) {
    case LEFT_FRONT:
      adc = analogRead(IR_LONG_1);
      //Serial.println(adc);
      temp = (6245.5)/(pow(adc,1.042));
      dist = MA_Filter(1, temp);
      break;
    case LEFT_BACK:
      adc = analogRead(IR_LONG_2);
      temp = (2730.4)/(pow(adc,0.901));
      dist = MA_Filter(2, temp);
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
     
      
  // might need delay dunno
  delay(1);
      
  return dist;
}

void StraightLineWithin(){
  if ((abs(error[0])<4 )){
    stop();
    Serial.println("Stopped"); 
  }
}

void stop() //Stop
{
  left_front_motor.writeMicroseconds(1500);
  left_rear_motor.writeMicroseconds(1500);
  right_rear_motor.writeMicroseconds(1500);
  right_front_motor.writeMicroseconds(1500);

  flag = 1;
}

double MA_Filter(int code, double distance){

  switch(code){
  case 1:
    sum_LFIR -= LEFT_FRONT_READING[index_LFIR];
    LEFT_FRONT_READING[index_LFIR] = distance;
    sum_LFIR += distance;
    index_LFIR = (index_LFIR + 1) % WINDOW_SIZE;
    averaged = sum_LFIR/WINDOW_SIZE;
//    Serial.println(averaged);
  break;
  case 2:
    sum_LBIR -= LEFT_BACK_READING[index_LBIR];
    LEFT_BACK_READING[index_LBIR] = distance;
    sum_LBIR += distance;
    index_LBIR = (index_LBIR + 1) % WINDOW_SIZE;
    averaged = sum_LBIR/WINDOW_SIZE;
//    Serial.println(averaged);
  break;
  }
  return averaged;
}
