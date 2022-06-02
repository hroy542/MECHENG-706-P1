#include <SoftwareSerial.h>
#include <SharpDistSensor.h>


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
// Left long range IR
const int IR_MID_1 = A4;
// Right long range IR
const int IR_MID_2 = A5;
// Front left mid range IR
const int IR_LONG_1 = A6;
// Front right mid range IR
const int IR_LONG_2 = A7;
const int GYRO = A8; //Gyro is attatched on pin A8
const int PTSensor1 = A9; //PTsensor1 is attached on pin A9
const int PTSensor2 = A10;
const int PTSensor3 = A12;
const int PTSensor4 = A13;//
const int trigPin = 34;
const int echoPin = 35;
const int MosfetPin = 45;

// Declare Variables
int ADC_sensor[10]; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO, [6]PTSensor1, [7]PTSensor2, [8]PTSensor3, [9]PTSensor4
float callibrated_sensor[10]; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO, [6]PTSensor1, [7]PTSensor2, [8]PTSensor3, [9]PTSensor
float temp = 0;
float Ultraduration;
float Ultradistance;
const float ultra_centre_offset = 11.5;
const int maxdist = 335;
const float mindist = 2.5;
unsigned long current_time = 0;
unsigned long previous_time = 0;
bool timer_first_call = true;
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
float sensor_past[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float last_var[6] = {999, 999, 999, 999, 999, 999}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float process_noise[6] = {5, 5, 5, 5, 10, 1}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float sensor_noise[6] = {6, 6, 6, 6, 50, 25}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
//----Kalman Filter----

//MA filter for the IR + Sonar sensors -------------
#define WINDOW_SIZE 13
int index[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float value[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float sum[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
float FRONT_LIR[WINDOW_SIZE];
float BACK_LIR[WINDOW_SIZE];
float LEFT_MIR[WINDOW_SIZE];
float RIGHT_MIR[WINDOW_SIZE];
float SONAR[5];
float averaged[6] = {0, 0, 0, 0, 0, 0}; //ORDER GOES; [0]LEFT LIR, [1]RIGHT LIR, [2]LEFT MIR, [3]RIGHT MIR, [4]SONAR, [5]GYRO
//MA Filter -------------


//Median Filtering using the sharp library -------------
const byte medianFilterWindowSize = 5;
const byte medianFilterWindowSize1 = 11;

SharpDistSensor LEFT_LONG(IR_LONG_1, medianFilterWindowSize);
SharpDistSensor RIGHT_LONG(IR_LONG_2, medianFilterWindowSize);
SharpDistSensor LEFT_MID(IR_MID_1, medianFilterWindowSize1);
SharpDistSensor RIGHT_MID(IR_MID_2, medianFilterWindowSize1);

const float C_L1 = 7058.6;
const float P_L1 = -1.066;
const float C_L2 = 11178;
const float P_L2 = -1.14;
const float C_M1 = 5586.2;
const float P_M1 = -1.15;
const float C_M2 = 452.63;
const float P_M2 = -0.728;

const unsigned int minVal_L1 = 40; // ~800 mm
const unsigned int maxVal_L1 = 500; // ~50mm
const unsigned int minVal_L2 = 130; // ~800 mm
const unsigned int maxVal_L2 = 500; // ~50mm
const unsigned int minVal_M1 = 90; // ~800 mm
const unsigned int maxVal_M1 = 410; // ~50mm
const unsigned int minVal_M2 = 35; // ~800 mm
const unsigned int maxVal_M2 = 400; // ~50mm

//Median Filtering using the sharp library -------------

//Battery Check Variables
int Lipo_level_cal;
int raw_lipo;

void setup() {
  Serial.begin(115200); // start serial communication
  BluetoothSerial.begin(115200);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(MosfetPin, OUTPUT);

  LEFT_LONG.setPowerFitCoeffs(C_L1, P_L1, minVal_L1, maxVal_L1);
  RIGHT_LONG.setPowerFitCoeffs(C_L2, P_L2, minVal_L2, maxVal_L2);
  LEFT_MID.setPowerFitCoeffs(C_M1, P_M1, minVal_M1, maxVal_M1);
  RIGHT_MID.setPowerFitCoeffs(C_M2, P_M2, minVal_M2, maxVal_M2);

  //----Gyro----
  int i;
  float summ = 0;

  for (i = 0; i < 100; i++) // read 100 values of voltage when gyro is at still, to calculate the zero-drift.
  {
    ADC_sensor[5] = analogRead(GYRO);
    summ += ADC_sensor[5];
    delay(5);
  }
  gyroZeroVoltage = summ / 100;  // average the sum as the zero drifting
  //----Gyro----
}

//###############################################################################################################################//
int wirelesscallibration = 1; // 1 to automatically select the mode selected below; 0 to choose the mode using serial monitor
//###############################################################################################################################//


void loop() {
  Serial.println("Please enter the mode you want to run");
  if (wirelesscallibration == 1) {
    Mode = 8;
  } else {
    Mode = GetSerialIN();
  }
  switch (Mode) {
    //USE MODE 1 FOR CALLIBRATION, PRINTS OUT THE RAW ADC INPUTS
    case 1:
      while (1) {
        ADC_sensor[0] = analogRead(IR_LONG_1); // the read out is a signal from 0-1023 corresponding to 0-5v
        ADC_sensor[1] = analogRead(IR_LONG_2);
        ADC_sensor[2] = analogRead(IR_MID_1);
        ADC_sensor[3] = analogRead(IR_MID_2);
        //Callibration code goes here
        Serial.print("LFL: ");
        Serial.print(ADC_sensor[0]);
        Serial.print("   |   ");
        Serial.print("RFL: ");
        Serial.print(ADC_sensor[1]);
        Serial.print("   |   ");
        Serial.print("LSM: ");
        Serial.print(ADC_sensor[2]);
        Serial.print("   |   ");
        Serial.print("RSM: ");
        Serial.println(ADC_sensor[3]);

        //Bluetooth Serial Prints
        BluetoothSerial.print("LFL: ");
        BluetoothSerial.print(ADC_sensor[0]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("RFL: ");
        BluetoothSerial.print(ADC_sensor[1]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("LSM: ");
        BluetoothSerial.print(ADC_sensor[2]);
        BluetoothSerial.print("   |   ");
        BluetoothSerial.print("RSM: ");
        BluetoothSerial.println(ADC_sensor[3]);
        delay(200);
      }
      break;

    //USE MODE 2 TO PRINT OUT THE RAW LIPO ADC VALUE
    case 2:
      while (1) {
        raw_lipo = analogRead(A0);
        Serial.println(raw_lipo);
        delay(400);
        BluetoothSerial.println(raw_lipo);
      }
      break;

    //USE MODE 3 TO PRINT OUT THE BATTERY PERCENTAGE LEVEL
    case 3:
      while (1) {
        raw_lipo = analogRead(A0);
        Lipo_level_cal = (raw_lipo - 717);
        Lipo_level_cal = Lipo_level_cal * 100;
        Lipo_level_cal = Lipo_level_cal / 143;
        Serial.print("Lipo Level: ");
        Serial.print(Lipo_level_cal);
        Serial.println("%");
        delay(400);
        BluetoothSerial.print("Lipo Level: ");
        BluetoothSerial.print(Lipo_level_cal);
        BluetoothSerial.println("%");
      }
      break;

    //USE MODE 4 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE FRONT LIR SENSOR
    case 4:
      while (1) {
        ADC_sensor[0] = analogRead(IR_LONG_1); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[0] != 0) {
          SensorSignalProcess(1, ADC_sensor[0]);
        }


      }
      break;

    //USE MODE 5 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK LIR SENSOR
    case 5:
      while (1) {
        ADC_sensor[1] = analogRead(IR_LONG_2); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[1] != 0) {
          SensorSignalProcess(2, ADC_sensor[1]);
        }

      }
      break;

    //USE MODE 6 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE LEFT MIR SENSOR
    case 6:
      while (1) {
        ADC_sensor[2] = analogRead(IR_MID_1); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[2] != 0) {
          SensorSignalProcess(3, ADC_sensor[2]);
        }

      }
      break;

    //USE MODE 7 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE RIGHT MIR SENSOR
    case 7:
      while (1) {
        ADC_sensor[3] = analogRead(IR_MID_2); // the read out is a signal from 0-1023 corresponding to 0-5v
        if (ADC_sensor[3] != 0) {
          SensorSignalProcess(4, ADC_sensor[3]);
        }
      }
      break;

    //USE MODE 8 TO PRINT ALL IR FILTERED VALUES AT ONCE
    case 8:
      while (1) {
      
//        Serial.print(C_L1* pow(analogRead(IR_LONG_1),P_L1));
//        Serial.print(", ");
//        Serial.println(LEFT_LONG.getDist());
//        Serial.print(C_L2* pow(analogRead(IR_LONG_2), P_L2));
//        Serial.print(", ");
//        Serial.println(RIGHT_LONG.getDist());
//        Serial.print(C_M1* pow(analogRead(IR_MID_1), P_M1));
//        Serial.print(", ");
//        Serial.println(LEFT_MID.getDist());
        Serial.print(C_M2* pow(analogRead(IR_MID_2), P_M2));
        Serial.print(", ");
        Serial.println(RIGHT_MID.getDist());
//        delay(5);
        

      }
      break;

    //USE MODE 9 TO CALLIBRATE THE GYRO
    case 9:
      while (1) {
        callibrated_sensor[5] = getGyroAngle();
        //      BluetoothSerial.print(Value1, DEC);
        //      BluetoothSerial.print(Delimiter);
        //      BluetoothSerial.print(Value2, DEC);
        //      BluetoothSerial.print(Delimiter);
        //      BluetoothSerial.println(Value3, DEC);
      }
      break;

    //USE MODE 10 TO PRINT FILTERED ULTRASONIC VALUES
    case 10:
      while (1) {
        current_time = millis();
        if ((current_time - previous_time) >= 60 || timer_first_call) {
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

    //USE MODE 11 TO CHECK ALL 4 PT SIGNALS TOGETHER
    case 11:
      while (1) {
        String Delimiter = " + ";
        ADC_sensor[6] = analogRead(PTSensor1);//LEFT MOST
        ADC_sensor[7] = analogRead(PTSensor2);
        ADC_sensor[8] = analogRead(PTSensor3);
        ADC_sensor[9] = analogRead(PTSensor4);//RIGHT MOST
        int PT_sum = ADC_sensor[6] + ADC_sensor[7] + ADC_sensor[8] + ADC_sensor[9];

        Serial.print(ADC_sensor[6]);
        Serial.print(Delimiter);
        Serial.print(ADC_sensor[7]);
        Serial.print(Delimiter);
        Serial.print(ADC_sensor[8]);
        Serial.print(Delimiter);
        Serial.print(ADC_sensor[9]);
        Serial.print(" = ");
        Serial.println(PT_sum);
        //WIRELESS PRINTING
        BluetoothSerial.print(ADC_sensor[6]);
        BluetoothSerial.print(Delimiter);
        BluetoothSerial.print(ADC_sensor[7]);
        BluetoothSerial.print(Delimiter);
        BluetoothSerial.print(ADC_sensor[8]);
        BluetoothSerial.print(Delimiter);
        BluetoothSerial.print(ADC_sensor[9]);
        BluetoothSerial.print(" = ");
        BluetoothSerial.println(PT_sum);
        delay(150);
      }
      break;

    //USE MODE 12 TO USE FAN
    case 12:
      while (1) {
        ADC_sensor[6] = analogRead(PTSensor1);
        //      BluetoothSerial.println(ADC_sensor[6]);

//        if (ADC_sensor[6] >= 985) {
//          while (ADC_sensor[6] >= 850) {
//            ADC_sensor[6] = analogRead(PTSensor1);
//            digitalWrite(MosfetPin, HIGH);
//          }
//        }
//        digitalWrite(MosfetPin, LOW);
        digitalWrite(MosfetPin, HIGH);
        delay(5000);
        digitalWrite(MosfetPin, LOW);

      }
      break;

    //USE MODE 13 TO CALLIBRATE THE MID RANGE IR SENSORS AT ONCE
    case 13:
      while (1) {
        serialFlush(); // waits for serial buffer to clear
        Serial.println("Please Enter the Distance away from the wall/obstacle");\
        int CaliDist;
        if(!wirelesscallibration){
          CaliDist = GetSerialIN();
        } else {
          CaliDist = 999;
        }
        int count = 0;
        int iter = 60;
        float RMidSum = 0;
        float RMidAvg = 0;
        float LMidSum = 0;
        float LMidAvg = 0;

        Serial.println("CALLIBRATING DO NOT MOVE. CURRENT CALLBIRATION DIST: " + String(CaliDist));
        while (count < iter) {
          ADC_sensor[3] = analogRead(IR_MID_2); // NOW THE RIGHT SIDE MID RANGE IR 
          ADC_sensor[2] = analogRead(IR_MID_1); // NOW THE LEFT SIDE MID RANGE IR
          RMidSum += ADC_sensor[3];
          LMidSum += ADC_sensor[2];
          count++;
          delay(15);
        }
        RMidAvg = RMidSum/iter;
        LMidAvg = LMidSum/iter;
        Serial.println("At " + String(CaliDist) + " cm, LEFT MID ADC: " + String(LMidAvg) + ", RIGHT MID ADC: " + String(RMidAvg));
        BluetoothSerial.println("At " + String(CaliDist) + " cm, LEFT MID ADC: " + String(LMidAvg) + ", RIGHT MID ADC: " + String(RMidAvg));
      }
      break;
      
    //USE MODE 14 TO CALLIBRATE THE LONG RANGE IR SENSORS AT ONCE
    case 14:
      while (1) {
        serialFlush(); // waits for serial buffer to clear
        Serial.println("Please Enter the Distance away from the wall/obstacle");
        int CaliDist;
        if(!wirelesscallibration){
          CaliDist = GetSerialIN();
        } else {
          CaliDist = 999;
        }        
        int count = 0;
        int iter = 60;
        float RLongSum = 0;
        float RLongAvg = 0;
        float LLongSum = 0;
        float LLongAvg = 0;

        Serial.println("CALLIBRATING DO NOT MOVE. CURRENT CALLBIRATION DIST: " + String(CaliDist));
        while (count < iter) {
          ADC_sensor[1] = analogRead(IR_LONG_2); // A5, NOW THE RIGHT FRONT LONG IR
          ADC_sensor[0] = analogRead(IR_LONG_1); // A4, NOW THE LEFT FRONT LONG IR
          RLongSum += ADC_sensor[1];
          LLongSum += ADC_sensor[0];
          count++;
          delay(15);
        }
        RLongAvg = RLongSum/iter;
        LLongAvg = LLongSum/iter;
        Serial.println("At " + String(CaliDist) + " cm, LEFT LONG ADC: " + String(LLongAvg) + ", RIGHT LONG ADC: " + String(RLongAvg));
        BluetoothSerial.println("At " + String(CaliDist) + " cm, LEFT LONG ADC: " + String(LLongAvg) + ", RIGHT LONG ADC: " + String(RLongAvg));
      }
      break;
    //USE MODE 15 TO CALLIBRATE THE PHOTOTRANSISTORS
    case 15:
      while (1) {
        serialFlush(); // waits for serial buffer to clear
        Serial.println("Please Enter the Distance away from the wall/obstacle");
        int CaliDist;
        if(!wirelesscallibration){
          CaliDist = GetSerialIN();
        } else {
          CaliDist = 999;
        }
        int count = 0;
        int iter = 60;
        float PT1Sum = 0;
        float PT2Sum = 0;
        float PT3Sum = 0;
        float PT4Sum = 0;
        float PT1Avg = 0;
        float PT2Avg = 0;
        float PT3Avg = 0;
        float PT4Avg = 0;
        float PTSum = 0;
        float PTLeft = 0;
        float PTRight = 0;


        Serial.println("CALLIBRATING DO NOT MOVE. CURRENT CALLBIRATION DIST: " + String(CaliDist));
        while (count < iter) {
          ADC_sensor[6] = analogRead(PTSensor1);//LEFT MOST
          ADC_sensor[7] = analogRead(PTSensor2);
          ADC_sensor[8] = analogRead(PTSensor3);
          ADC_sensor[9] = analogRead(PTSensor4);//RIGHT MOST
          PT1Sum += ADC_sensor[6];
          PT2Sum += ADC_sensor[7];
          PT3Sum += ADC_sensor[8];
          PT4Sum += ADC_sensor[9];
          PTLeft = PT1Sum + PT2Sum;
          PTRight = PT3Sum + PT4Sum;
          PTSum = PTLeft + PTRight;
          count++;
          delay(15);
        }
        PT1Avg = PT1Sum/iter;
        PT2Avg = PT2Sum/iter;
        PT3Avg = PT3Sum/iter;
        PT4Avg = PT4Sum/iter;
        PTSum = PTSum/iter;
        PTLeft = PTLeft/iter;
        PTRight = PTRight/iter;
        Serial.println("At " + String(CaliDist) + " cm, PT1(LEFT MOST): " + String(PT1Avg) + ", PT2: " + String(PT2Avg) + ", PT3: " + String(PT3Avg) + ", PT4(RIGHT MOST): " + String(PT4Avg) );
        Serial.println("PTLEFT: " + String(PTLeft) + "+ PTRight: " + String(PTRight) + " =  PTSum: " + String(PTSum));
        BluetoothSerial.println("At " + String(CaliDist) + " cm, PT1(LEFT MOST): " + String(PT1Avg) + ", PT2: " + String(PT2Avg) + ", PT3: " + String(PT3Avg) + ", PT4(RIGHT MOST): " + String(PT4Avg) );
        BluetoothSerial.println("PTLEFT: " + String(PTLeft) + "+ PTRight: " + String(PTRight) + " =  PTSum: " + String(PTSum));

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

float getGyroAngle() {
  ADC_sensor[5] = analogRead(GYRO);
  angularVelocity = ((ADC_sensor[5] - gyroZeroVoltage) * gyroSupplyVoltage) / (1023 * gyroSensitivity);
  if (angularVelocity >= rotationThreshold || angularVelocity <= -rotationThreshold)
  {
    currentAngle += angularVelocity / (1000 / T);
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
  switch (code) {
    case 1:
      dist = 5780.3 * pow(RawADC, -1.027); //Callibration for Front LIR

      //    ----KALMAN FILTER
      callibrated_sensor[0] = Kalman(dist, sensor_past[0], process_noise[0], sensor_noise[0], last_var[0], 1);
      //      sensor_past[0] = callibrated_sensor[0];
      //    ----KALMAN FILTER

      //    ----MA FILTER
      sum[0] -= FRONT_LIR[index[0]];
      FRONT_LIR[index[0]] = callibrated_sensor[0];
      sum[0] += callibrated_sensor[0];
      index[0] = (index[0] + 1) % WINDOW_SIZE;
      averaged[0] = sum[0] / WINDOW_SIZE;
      sensor_past[0] = averaged[0];
      //    ----MA FILTER
//
//      Serial.print(dist);
//      Serial.print(",");
//      Serial.println(averaged[0]);
      delay(5);
      break;
    case 2:

      dist = 4382.9 * pow(RawADC, -.984); //Callibration for Back LIR

      //    ----KALMAN FILTER
      callibrated_sensor[1] = Kalman(dist, sensor_past[1], process_noise[1], sensor_noise[1], last_var[1], 2);
      //      sensor_past[1] = callibrated_sensor[1];
      //    ----KALMAN FILTER

      //    ----MA FILTER
      sum[1] -= BACK_LIR[index[1]];
      BACK_LIR[index[1]] = callibrated_sensor[1];
      sum[1] += callibrated_sensor[1];
      index[1] = (index[1] + 1) % WINDOW_SIZE;
      averaged[1] = sum[1] / WINDOW_SIZE;
      sensor_past[1] = averaged[1];
      //    ----MA FILTER

//      Serial.print(dist);
//      Serial.print(",");
//      Serial.println(averaged[1]);
      delay(5);

      break;
    case 3:

      dist = 3730.6 * pow(RawADC, -1.082); //Callibration for LEFT MIR

      //    ----KALMAN FILTER
      callibrated_sensor[2] = Kalman(dist, sensor_past[2], process_noise[2], sensor_noise[2], last_var[2], 3);
      //      sensor_past[2] = callibrated_sensor[2];
      //    ----KALMAN FILTER

      //    ----MA FILTER
      sum[2] -= BACK_LIR[index[2]];
      BACK_LIR[index[2]] = callibrated_sensor[2];
      sum[2] += callibrated_sensor[2];
      index[2] = (index[2] + 1) % WINDOW_SIZE;
      averaged[2] = sum[2] / WINDOW_SIZE;
      sensor_past[2] = averaged[2];
      //    ----MA FILTER

//      Serial.print(dist);
//      Serial.print(",");
//      Serial.println(averaged[2]);
      delay(5);

      break;
    case 4:

      dist = 3491.3 * pow(RawADC, -1.069); //Callibration for RIGHT MIR

      //    ----KALMAN FILTER
      callibrated_sensor[3] = Kalman(dist, sensor_past[3], process_noise[3], sensor_noise[3], last_var[3], 4);
      sensor_past[3] = callibrated_sensor[3];
      //    ----KALMAN FILTER

      //    ----MA FILTER
      sum[3] -= BACK_LIR[index[3]];
      BACK_LIR[index[3]] = callibrated_sensor[3];
      sum[3] += callibrated_sensor[3];
      index[3] = (index[3] + 1) % WINDOW_SIZE;
      averaged[3] = sum[3] / WINDOW_SIZE;
      sensor_past[3] = averaged[3];
      //    ----MA FILTER

//      Serial.print(dist);
//      Serial.print(",");
//      Serial.println(averaged[3]);
      delay(5);

      break;
    case 5:

      callibrated_sensor[4] = ultra_centre_offset + (RawADC * .034) / 2;

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
      averaged[4] = sum[4] / 5;
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

float Sonarkalman(float U){
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
}

float Kalman(float rawdata, float prev_est, float process_noise, float sensor_noise, float last_variance, int sensor) {  // Kalman Filter
  float a_priori_est, a_post_est, a_priori_var, kalman_gain;
  float a_post_var;

  a_priori_est = prev_est;
  a_priori_var = last_variance + process_noise;
  kalman_gain = a_priori_var / (a_priori_var + sensor_noise);
  a_post_est = a_priori_est + kalman_gain * (rawdata - a_priori_est);
  a_post_var = (1 - kalman_gain) * a_priori_var;

  switch (sensor) {
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

int GetSerialIN() {
  while (Serial.available() == 0) {
  }
  return Serial.parseInt();
}
void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

// random link https://www.norwegiancreations.com/2016/01/tutorial-multiple-values-in-the-arduino-ide-serial-plotter/
