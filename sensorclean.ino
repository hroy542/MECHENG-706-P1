#include <SoftwareSerial.h>

#define INTERNAL_LED 13

// Serial Data input pin
#define BLUETOOTH_RX 10
// Serial Data output pin
#define BLUETOOTH_TX 11
// Bluetooth Serial Port
#define OUTPUTBLUETOOTHMONITOR 1
volatile int32_t Counter = 1;
String Delimiter = ", ";

SoftwareSerial BluetoothSerial(BLUETOOTH_RX, BLUETOOTH_TX);

// Declare Pins
const int LEFT_FRONT_LIR = A4; //sensor is attatched on pin A4
const int LEFT_BACK_LIR = A5; //sensor is attatched on pin A5
const int LEFT_BACK_MIR = A6; //sensor is attatched on pin A6
const int RIGHT_BACK_MIR = A7; //sensor is attatched on pin A7
const int GYRO = A8; //Gyro is attatched on pin A8
const int trigPin = 34;
const int echoPin = 35;

// Declare Variables
int ADC_sensor[6]; //ORDER GOES; [1]FRONT LIR, [2]BACK LIR, [3]LEFT MIR, [4]RIGHT MIR, [5]SONAR, [6]GYRO
float callibrated_sensor[6]; //ORDER GOES; [1]FRONT LIR, [2]BACK LIR, [3]LEFT MIR, [4]RIGHT MIR, [5]SONAR, [6]GYRO
float temp = 0;
float Ultraduration;
float Ultradistance;
const float ultra_centre_offset = 11.5;
const int maxdist = 335;
const float mindist = 2.5;
unsigned long current_time = 0;
unsigned long previous_time = 0;
bool timer_first_call = true;
int wirelesscallibration = 1;
int Mode = 1;

///*----GYRO Variables----
float EMA_a = 0.4;
float EMA_S = 0;
int T = 100;               // T is the time of one loop, 0.05 sec
float gyroSupplyVoltage = 5;      // supply voltage for gyro
float gyroZeroVoltage = 0;         // the value of voltage when gyro is zero
float gyroSensitivity = 0.0057;     // gyro sensitivity unit is (mv/degree/second) get from datasheet
float rotationThreshold = 0;      // because of gyro drifting, defining rotation angular velocity less
// than this value will be ignored
float currentAngle = 0;               // current angle calculated by angular velocity integral on
float radiansAngle = 0;
float angularVelocity = 0;
//----GYRO Variable----*/

//----Kalman Filter----
float sensor_past[6] = {0,0,0,0,0,0}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float last_var[6] = {999,999,999,999,999,999}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float process_noise[6] = {5,5,5,5,10,1}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float sensor_noise[6] = {5,5,5,5,50,25}; //ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
//----Kalman Filter----

//MA filter for the IR + Sonar sensors -------------
#define WINDOW_SIZE 13
int index[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float value[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float sum[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float FRONT_LIR[WINDOW_SIZE];
float BACK_LIR[WINDOW_SIZE];
float LEFT_MIR[WINDOW_SIZE];
float RIGHT_MIR[WINDOW_SIZE];
float SONAR[5];
float averaged[6] = {0,0,0,0,0,0};//ORDER GOES; [0]FRONT LIR, [1]BACK LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
//MA Filter -------------

//Battery Check Variables
int Lipo_level_cal;
int raw_lipo;

void setup() {
  Serial.begin(115200); // start serial communication
  
  BluetoothSerial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //----Gyro----
  int i;
  float summ = 0;
  
  Serial.println("please keep the sensor still for calibration");
  Serial.println("get the gyro zero voltage");
  for (i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    ADC_sensor[6] = analogRead(GYRO);
    summ += ADC_sensor[6];
    delay(5);
  }
  gyroZeroVoltage = summ / 100;  // average the sum as the zero drifting
  //----Gyro----
}


void loop(){
  Serial.println("Please enter the mode you want to run: Battery Raw [2], Battery Percentage [3] or Enter the Sensor pin for the Sensor you want to tune [4 - 7]?:");


  
  if (wirelesscallibration == 1){
    Mode = 1;
  } else {
    while (Serial.available() ==0){
    }
    Mode = Serial.parseInt();
  }
  switch (Mode){
    //USE MODE 1 FOR CALLIBRATION, PRINTS OUT THE RAW ADC INPUTS
    case 1:
      while(1){
        ADC_sensor[0] = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        ADC_sensor[1] = analogRead(LEFT_BACK_LIR);
        ADC_sensor[2] = analogRead(LEFT_BACK_MIR);
        ADC_sensor[3] = analogRead(RIGHT_BACK_MIR);
        //Callibration code goes here
        Serial.print("LFL: ");
        Serial.print(ADC_sensor[0]);
        Serial.print("   |   ");
        Serial.print("LBL: ");
        Serial.print(ADC_sensor[1]);
        Serial.print("   |   ");
        Serial.print("LBM: ");
        Serial.print(ADC_sensor[2]);
        Serial.print("   |   ");
        Serial.print("RBM: ");
        Serial.println(ADC_sensor[3]);

        //Bluetooth Serial Prints
        BluetoothSerial.print("LFL: ");
        BluetoothSerial.print(ADC_sensor[0]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("LBL: ");
        BluetoothSerial.print(ADC_sensor[1]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("LBM: ");
        BluetoothSerial.print(ADC_sensor[2]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("RBM: ");
        BluetoothSerial.println(ADC_sensor[3]);
        delay(200);
    }
    break;

    //USE MODE 2 TO PRINT OUT THE RAW LIPO ADC VALUE    
    case 2:
    while(1){
      raw_lipo=analogRead(A0);
      Serial.println(raw_lipo);
      delay(400);
      
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
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
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
    }
    break;

    //USE MODE 4 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE FRONT LIR SENSOR
    case 4:
      while(1){
        ADC_sensor[0] = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[0] != 0){
          SensorSignalProcess(1, ADC_sensor[0]);
        }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);      
      }
    break;

    //USE MODE 5 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK LIR SENSOR
    case 5:
      while(1){
        ADC_sensor[1] = analogRead(LEFT_BACK_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[1] != 0){
          SensorSignalProcess(2, ADC_sensor[1]);
        }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
      }
    break;

    //USE MODE 6 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE LEFT MIR SENSOR
    case 6:
      while(1){
        ADC_sensor[2] = analogRead(LEFT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[2] != 0){
          SensorSignalProcess(3, ADC_sensor[2]);        
        }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
      }
    break;

    //USE MODE 7 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE RIGHT MIR SENSOR
    case 7:
      while(1){
        ADC_sensor[3] = analogRead(RIGHT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        if(ADC_sensor[3] != 0){
          SensorSignalProcess(4, ADC_sensor[3]);        
        }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
      }
    break;

    //USE MODE 8 TO PRINT BOTH LONG RANGE IR FILTERED VALUES AT ONCE
    case 8:
      while(1){
        ADC_sensor[0] = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        SensorSignalProcess(1, ADC_sensor[0]);
        ADC_sensor[1] = analogRead(LEFT_BACK_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        SensorSignalProcess(2, ADC_sensor[1]);
//        Comment out the serial prints in the SensorSignalProcess function
//        Serial.print(averaged[0]);
//        Serial.print(", ");
//        Serial.println(averaged[1]);
        
      }
    break;

    //USE MODE 9 TO PRINT BOTH MID RANGE IR FILTERED VALUES AT ONCE
    case 9:
      while(1){

      }
    break;

    //USE MODE 10 TO CALLIBRATE THE GYRO
    case 10:
    while(1){
      callibrated_sensor[5] = getGyroAngle();
    }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
    break;

    //USE MODE 11 TO PRINT FILTERED ULTRASONIC VALUES
    case 11:
    while(1){
      current_time = millis();
      if((current_time-previous_time)>= 60 || timer_first_call){
        previous_time = current_time;
        ADC_sensor[4] = usonic_transmit();
        SensorSignalProcess(5, ADC_sensor[4]); 

        timer_first_call = false;
      }
//      BluetoothSerial.print(Value1, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.print(Value2, DEC);
//      BluetoothSerial.print(Delimiter);
//      BluetoothSerial.println(Value3, DEC);
    } 
    break;
  }
}

float usonic_transmit() {
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  float duration = pulseIn(echoPin, HIGH);
  return duration;
  
}

float getGyroAngle(){
  ADC_sensor[5] = analogRead(GYRO);
  angularVelocity = ((ADC_sensor[5] - gyroZeroVoltage) * gyroSupplyVoltage)/(1023 * gyroSensitivity);
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    currentAngle += angularVelocity/(1000/T);
  }

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
  return currentAngle;
}

void SensorSignalProcess(int code, float RawADC) { // find distances using calibration curve equations
  float dist;
  switch(code) {
    case 1:
      dist = 5780.3*pow(RawADC, -1.027);//Callibration for Front LIR
      
//    ----KALMAN FILTER
      callibrated_sensor[0] = Kalman(dist, sensor_past[0], process_noise[0], sensor_noise[0], last_var[0], 1);
//      sensor_past[0] = callibrated_sensor[0];
//    ----KALMAN FILTER

//    ----MA FILTER
      sum[0] -= FRONT_LIR[index[0]];
      FRONT_LIR[index[0]] = callibrated_sensor[0];
      sum[0] += callibrated_sensor[0];
      index[0] = (index[0] + 1) % WINDOW_SIZE;
      averaged[0] = sum[0]/WINDOW_SIZE;
      sensor_past[0] = averaged[0];
//    ----MA FILTER
                  
      Serial.print(dist);
      Serial.print(",");
      Serial.println(averaged[0]);
      delay(5);
    break;
    case 2:
    
      dist = 4382.9*pow(RawADC, -.984);//Callibration for Back LIR

//    ----KALMAN FILTER
      callibrated_sensor[1] = Kalman(dist, sensor_past[1], process_noise[1], sensor_noise[1], last_var[1], 2);
//      sensor_past[1] = callibrated_sensor[1];
//    ----KALMAN FILTER

//    ----MA FILTER
      sum[1] -= BACK_LIR[index[1]];
      BACK_LIR[index[1]] = callibrated_sensor[1];
      sum[1] += callibrated_sensor[1];
      index[1] = (index[1] + 1) % WINDOW_SIZE;
      averaged[1] = sum[1]/WINDOW_SIZE;
      sensor_past[1] = averaged[1];
//    ----MA FILTER

      Serial.print(dist);
      Serial.print(",");
      Serial.println(averaged[1]);
      delay(5);

    break;
    case 3:
    
      dist = 3730.6*pow(RawADC, -1.082);//Callibration for LEFT MIR

//    ----KALMAN FILTER
      callibrated_sensor[2] = Kalman(dist, sensor_past[2], process_noise[2], sensor_noise[2], last_var[2], 3);
//      sensor_past[2] = callibrated_sensor[2];
//    ----KALMAN FILTER

//    ----MA FILTER
      sum[2] -= BACK_LIR[index[2]];
      BACK_LIR[index[2]] = callibrated_sensor[2];
      sum[2] += callibrated_sensor[2];
      index[2] = (index[2] + 1) % WINDOW_SIZE;
      averaged[2] = sum[2]/WINDOW_SIZE;
      sensor_past[2] = averaged[2];
//    ----MA FILTER

      Serial.print(dist);
      Serial.print(",");
      Serial.println(averaged[2]);
      delay(5);
      
    break;
    case 4:
    
      dist = 3491.3*pow(RawADC, -1.069);//Callibration for RIGHT MIR

//    ----KALMAN FILTER
      callibrated_sensor[3] = Kalman(dist, sensor_past[3], process_noise[3], sensor_noise[3], last_var[3], 4);
      sensor_past[3] = callibrated_sensor[3];
//    ----KALMAN FILTER

//    ----MA FILTER
      sum[3] -= BACK_LIR[index[3]];
      BACK_LIR[index[3]] = callibrated_sensor[3];
      sum[3] += callibrated_sensor[3];
      index[3] = (index[3] + 1) % WINDOW_SIZE;
      averaged[3] = sum[3]/WINDOW_SIZE;
      sensor_past[3] = averaged[3];
//    ----MA FILTER

      Serial.print(dist);
      Serial.print(",");
      Serial.println(averaged[3]);
      delay(5);

    break;
    case 5:

      callibrated_sensor[4] = ultra_centre_offset + (RawADC*.034)/2;
      
////    ----KALMAN FILTER
//      float Ultdist = Kalman(callibrated_sensor[5], sensor_past[5], process_noise[5], sensor_noise[5], last_var[5], 5);
//      sensor_past[5] = callibrated_sensor[5];
//      float Ultradist = Sonarkalman(callibrated_sensor[5]);
////    ----KALMAN FILTER

//    ----MA FILTER
      sum[4] -= SONAR[index[4]];
      SONAR[index[4]] = callibrated_sensor[4];
      sum[4] += callibrated_sensor[4];
      index[4] = (index[4] + 1) % 5;
      averaged[4] = sum[4]/5;
//    ----MA FILTER
      Serial.print(callibrated_sensor[4]);
      Serial.print(",");
      Serial.println(averaged[4]);
      delay(5);

    break;
    case 6:

    break;
  }
}

/*float Sonarkalman(float U){
  static const float R = 10;
  static const float H = 1.00;
  static float Q = 1;
  static float P = 0;
  static float U_hat = 0;
  static float K = 0;
  K = P*H/(H*P*H+R);
  U_hat += + K*(U-H*U_hat);
  P = (1-K*H)*P+Q;
  return U_hat;
}*/

float Kalman(float rawdata, float prev_est, float process_noise, float sensor_noise, float last_variance, int sensor){   // Kalman Filter
  float a_priori_est, a_post_est, a_priori_var, kalman_gain;
  float a_post_var;

  a_priori_est = prev_est;
  a_priori_var = last_variance + process_noise;
  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  
  switch(sensor) {
    case 1:
      last_var[0] = a_post_var;
    break;
    case 2:
      last_var[1] = a_post_var;
    break;
    case 3:
      last_var[2] = a_post_var;
    break;
    case 4:
      last_var[3] = a_post_var;
    break;
    case 5:
      last_var[4] = a_post_var;
    break;
    case 6:
      last_var[5] = a_post_var;
    break;
  } 
  return a_post_est;
}

// random link https://www.norwegiancreations.com/2016/01/tutorial-multiple-values-in-the-arduino-ide-serial-plotter/    
