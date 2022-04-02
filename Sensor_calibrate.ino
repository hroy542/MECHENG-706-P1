#include <SharpIR.h>

SharpIR LEFT_BACK_MIR(SharpIR::GP2Y0A41SK0F, A6);
SharpIR RIGHT_BACK_MIR(SharpIR::GP2Y0A41SK0F, A7);


// Declare Pins
const int LEFT_FRONT_LIR = A4; //sensor is attatched on pin A4
const int LEFT_BACK_LIR = A5; //sensor is attatched on pin A5
//const int LEFT_BACK_MIR = A6; //sensor is attatched on pin A6
//const int RIGHT_BACK_MIR = A7; //sensor is attatched on pin A7
const int GYRO = A8; //Gyro is attatched on pin A8
const int trigPin = 34;
const int echoPin = 35;
double Ultraduration;
double Ultradistance;
const double ultra_centre_offset = 11.5;
const int maxdist = 335;
const float mindist = 2.5;

//byte serialRead = 0; //for control serial communication 

// Declare Variables
int Analogue4 = 0; // the read out signal in 0-1023 corresponding to 0-5V
int Analogue5 = 0; 
int Analogue6 = 0; 
int Analogue7 = 0;
int Analogue8 = 0;
double distance4 = 0; //for the calculated distance using the sensor callibrations
double distance5 = 0;
double distance6 = 0;
double distance7 = 0;
double Ultdist = 0;
double angle8 = 0;
double temp = 0;

//----GYRO Variables----
float EMA_a = 0.4;
double EMA_S = 0;
int T = 100;               // T is the time of one loop, 0.05 sec
float gyroSupplyVoltage = 5;      // supply voltage for gyro
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero
float gyroSensitivity = 0.0057;     // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 5;      // because of gyro drifting, defining rotation angular velocity less
// than this value will be ignored
double currentAngle = 0;               // current angle calculated by angular velocity integral on
double radiansAngle = 0;
double angularVelocity = 0;
//----GYRO Variable----

//----Kalman Filter----
double Distance4_Past = 0;
double Distance5_Past = 0;
double Distance6_Past = 0;
double Distance7_Past = 0;
double Angle8_Past = 0;
double last_var4 = 9999;
double last_var5 = 9999;
double last_var6 = 9999;
double last_var7 = 9999;
double last_var8 = 9999;
double process_noise4 = 5;
double process_noise5 = 1;
double process_noise6 = 1;
double process_noise7 = 1;
double process_noise8 = 1;
double sensor_noise4 = 5;
double sensor_noise5 = 25;
double sensor_noise6 = 25;
double sensor_noise7 = 25;
double sensor_noise8 = 25;
//----Kalman Filter----

//MA Filter -------------
#define WINDOW_SIZE 13
int index = 0;
double value = 0;
double sum = 0;
double READINGS[WINDOW_SIZE];
double averaged = 0;
//MA Filter -------------

//Battery Check Variables
int Lipo_level_cal;
int raw_lipo;

void setup() {
  Serial.begin(115200); // start serial communication
  //pinMode(LEFT_FRONT_LIR, INPUT);
  //pinMode(LEFT_BACK_LIR, INPUT);
  //pinMode(LEFT_BACK_MIR, INPUT);
  //pinMode(RIGHT_BACK_MIR, INPUT);
  pinMode(GYRO, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //----Gyro----
  int i;
  float sum = 0;
  EMA_S = analogRead(GYRO);     //set EMA S for t=1
  
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    Analogue8 = analogRead(GYRO);
    sum += Analogue8;
    delay(5);
  }
  gyroZeroVoltage = sum / 100;  // average the sum as the zero drifting
  //----Gyro----
}


void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println("Please enter the mode you want to run: Battery Raw [2], Battery Percentage [3] or Enter the Sensor pin for the Sensor you want to tune [4 - 7]?:");
  
  while (Serial.available() ==0){
  }
  int Mode = Serial.parseInt();

  switch (Mode){
    //USE MODE 1 FOR CALLIBRATION, PRINTS OUT THE RAW ADC INPUTS
    case 1:
      while(1){
        Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        Analogue5 = analogRead(LEFT_BACK_LIR);
//        Analogue6 = analogRead(LEFT_BACK_MIR);
//        Analogue7 = analogRead(RIGHT_BACK_MIR);
        
        //Callibration code goes here
        Serial.print("LFL: ");
        Serial.print(Analogue4);
        Serial.print("   |   ");
        Serial.print("LBL: ");
        Serial.print(Analogue5);
        Serial.print("   |   ");
        Serial.print("LBM: ");
        Serial.print(Analogue6);
        Serial.print("   |   ");
        Serial.print("RBM: ");
        Serial.println(Analogue7);
        delay(500);
    }
    break;

    //USE MODE 2 TO PRINT OUT THE RAW LIPO ADC VALUE    
    case 2:
    while(1){
      raw_lipo=analogRead(A0);
      Serial.println(raw_lipo);
      delay(400);
    }
    break;

    //USE MODE 3 TO PRINT OUT THE BATTERY PERCENTAGE LEVEL
    case 3:
    while(1){
      raw_lipo=analogRead(A0);
      Lipo_level_cal = (raw_lipo - 717);
      Lipo_level_cal = Lipo_level_cal * 100;
      Lipo_level_cal = Lipo_level_cal / 143;
      Serial.print("Lipo Level: ");
      Serial.print(Lipo_level_cal);
      Serial.println("%");
      delay(400);
    }
    break;

    //USE MODE 4 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE FRONT LEFT LIR SENSOR
    case 4:
      while(1){
        Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        temp = Distance(4, Analogue4);
        Serial.print(temp);
        Serial.print(",");

//      ----KALMAN FILTER
        distance4 = Kalman(temp, Distance4_Past, process_noise4, sensor_noise4, last_var4, 4);
//        Serial.println(distance4);
        Distance4_Past = distance4;
        delay(25);
//      ----KALMAN FILTER

//      ----MA FILTER
        sum -= READINGS[index];
        READINGS[index] = distance4;
        sum += distance4;
        index = (index + 1) % WINDOW_SIZE;
        averaged = sum/WINDOW_SIZE;
        Serial.println(averaged);
//      ----MA FILTER
        delay(5);
      }
    break;

    //USE MODE 5 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK LEFT LIR SENSOR
    case 5:
      while(1){
        Analogue5 = analogRead(LEFT_BACK_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        temp = Distance(5, Analogue5);
        Serial.print(temp);
        Serial.print(" , ");
        distance5 = Kalman(temp, Distance5_Past, process_noise5, sensor_noise5, last_var5, 5);
        Serial.println(distance5);
        Distance5_Past = distance5;
        delay(25);
      }
    break;

    //USE MODE 6 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK LEFT MIR SENSOR
    case 6:
      while(1){
//        Analogue6 = analogRead(LEFT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
//        temp = Distance(6, Analogue6);
        Serial.print(temp);
        Serial.print(" , ");
//        distance6 = Kalman(temp, Distance6_Past, process_noise6, sensor_noise6, last_var6, 6);
        Serial.println(distance6);
//        Distance6_Past = distance6;
        delay(25);

      }
    break;

    //USE MODE 7 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK RIGHT MIR SENSOR
    case 7:
      while(1){
//        Analogue7 = analogRead(RIGHT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
//        temp = Distance(7, Analogue7);
        Serial.print(temp);
        Serial.print(" , ");
//        distance7 = Kalman(temp, Distance7_Past, process_noise7, sensor_noise7, last_var7 ,7);
        Serial.println(distance7);
//        Distance7_Past = distance7;
        delay(25);
      }
    break;

    //USE MODE 8 TO PRINT BOTH LONG RANGE IR FILTERED VALUES AT ONCE
    case 8:
      while(1){
        Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        Analogue5 = analogRead(LEFT_BACK_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v

        temp = Distance(4, Analogue4);
        distance4 = Kalman(temp, Distance4_Past, process_noise4, sensor_noise4, last_var4, 4);
        Distance4_Past = distance4;

        Serial.print(distance4);
        Serial.print(",");

        temp = Distance(5, Analogue5);
        distance5 = Kalman(temp, Distance5_Past, process_noise5, sensor_noise5, last_var5, 5);
        Serial.println(distance5);
        Distance5_Past = distance5;
        delay(25);
      }
    break;

    //USE MODE 8 TO PRINT BOTH LONG RANGE IR FILTERED VALUES AT ONCE
    case 9:
      while(1){
//        Analogue6 = analogRead(LEFT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
//        Analogue7 = analogRead(RIGHT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v

//        temp = Distance(6, Analogue6);
//        distance6 = Kalman(temp, Distance6_Past, process_noise6, sensor_noise6, last_var6, 6);
//        Distance6_Past = distance6;

        Serial.print(distance6);
        Serial.print(",");

//        temp = Distance(7, Analogue7);
//        distance7 = Kalman(temp, Distance7_Past, process_noise7, sensor_noise7, last_var7, 7);
        Serial.println(distance7);
//        Distance7_Past = distance7;
        delay(25);
      }
    break;

    //USE MODE 10 TO CALLIBRATE THE GYRO
    case 10:
    while(1){
      angle8 = getGyroAngle();
    }
    break;

    //USE MODE 11 TO PRINT FILTERED ULTRASONIC VALUES
    case 11:
    while(1){

      Ultraduration = usonic_transmit();
      Ultradistance = ultra_centre_offset + (Ultraduration*.034)/2;
      Ultdist = Sonarkalman(Ultradistance);

      Serial.print(Ultradistance);
      Serial.print(",");
      Serial.println(Ultdist);
      delay(1/30);
    }
    break;
  }
}

double Sonarkalman(double U){
  static const double R = 40;
  static const double H = 1.00;
  static double Q = 10;
  static double P = 0;
  static double U_hat = 0;
  static double K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}

double usonic_transmit() {
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  double dur = pulseIn(echoPin, HIGH);
  Serial.println(dur);
  return dur;
}

double getGyroAngle(){
  Analogue8 = analogRead(GYRO);
  angularVelocity = ((Analogue8 - gyroZeroVoltage) * gyroSupplyVoltage)/(1023 * gyroSensitivity);
//      Serial.print(Analogue8);
//      Serial.print(", ");
//      Serial.println(angularVelocity);
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    // we are running a loop in T (of T/1000 second).
    currentAngle += angularVelocity/(1000/T);
  }
    Serial.println(currentAngle);
  if (currentAngle < 0)
  {
    currentAngle += 360;
  }
  else if (currentAngle > 359)
  {
    currentAngle -= 360;
  }

//  Serial.println(currentAngle);
  delay (T);
}

double Distance(int code, int RawADC) { // find distances using calibration curve equations
  double dist;
  
  switch(code) {
    case 1 ... 4:
      dist = 6245.5*pow(RawADC, -1.042);//Callibration for Left Front
//            dist = 2730.4*pow(RawADC, -.901);//Callibration for Left Back

//      dist = 4E-14*pow(RawADC,6) -8E-11*pow(RawADC,5) + 7E-08*pow(RawADC,4) - 3E-05*pow(RawADC,3) + 0.0085*pow(RawADC,2) - 1.262*RawADC + 108.47;
    break;
    case 5:
      dist = 2730.4*pow(RawADC, -.901);//Callibration for Left Back
//               dist = 6245.5*pow(RawADC, -1.042);//Callibration for Left Front

//      dist = -3E-11*pow(RawADC,5) + 4E-08*pow(RawADC,4) - 2E-05*pow(RawADC,3) + 0.0072*pow(RawADC,2) - 1.1758*RawADC + 106.25;
    break;
    case 6:
      dist = 1790.5*pow(RawADC, -.829);
//      dist = -3E-11*pow(RawADC,5) + 4E-08*pow(RawADC,4) - 2E-05*pow(RawADC,3) + 0.0072*pow(RawADC,2) - 1.1758*RawADC + 106.25;
    break;
    case 7:
      dist = 1790.5*pow(RawADC, -.829);
//      dist = -3E-11*pow(RawADC,5) + 4E-08*pow(RawADC,4) - 2E-05*pow(RawADC,3) + 0.0072*pow(RawADC,2) - 1.1758*RawADC + 106.25;
    break;
  }
  return dist;
}

//Kalman Filter for IR sensors
double Kalman(double rawdata, double prev_est, double process_noise, double sensor_noise, double last_var, int sensor){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;
  a_priori_var = last_var + process_noise;

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  
  switch(sensor) {
    case 1 ... 4:
      last_var4 = a_post_var;
    break;
    case 5:
      last_var5 = a_post_var;
    break;
    case 6:
      last_var6 = a_post_var;
    break;
    case 7:
      last_var7 = a_post_var;
    break;
  }
    
  return a_post_est;
}


// random link https://www.norwegiancreations.com/2016/01/tutorial-multiple-values-in-the-arduino-ide-serial-plotter/    
