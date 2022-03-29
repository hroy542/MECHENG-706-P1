// Declare Pins
int LEFT_FRONT_LIR = A4; //sensor is attached on pinA4
int LEFT_BACK_LIR = A5; //sensor is attached on pinA5
int LEFT_BACK_MIR = A6; //sensor is attached on pinA6
int RIGHT_BACK_MIR = A7; //sensor is attached on pinA7
//byte serialRead = 0; //for control serial communication 

// Declare Variables
int Analogue4 = 0; // the read out signal in 0-1023 corresponding to 0-5V
int Analogue5 = 0; 
int Analogue6 = 0; 
int Analogue7 = 0;
double distance4 = 0; //for the calculated distance using the sensor callibrations
double distance5 = 0;
double distance6 = 0;
double distance7 = 0;
double temp = 0;

//----Kalman Filter----
double Distance4_Past = 0;
double Distance5_Past = 0;
double Distance6_Past = 0;
double Distance7_Past = 0;
double last_var4 = 9999;
double last_var5 = 9999;
double last_var6 = 9999;
double last_var7 = 9999;
double process_noise4 = 1;
double process_noise5 = 1;
double process_noise6 = 1;
double process_noise7 = 1;
double sensor_noise4 = 25;
double sensor_noise5 = 25;
double sensor_noise6 = 25;
double sensor_noise7 = 25;// Change the value of sensor noise to get different KF performance
//----Kalman Filter----


//Battery Check Variables
int Lipo_level_cal;
int raw_lipo;

void setup() {
Serial.begin(115200); // start serial communication

}
void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.println("Please enter the mode you want to run: Callibration Mode [1], Battery Raw [2], Battery Percentage [3] or Enter the Sensor pin for the Sensor you want to tune [4 - 7]?:");
  
  while (Serial.available() ==0){
  }
  int Mode = Serial.parseInt();

  switch (Mode){
    //USE MODE 1 FOR CALLIBRATION, PRINTS OUT THE RAW ADC INPUTS
    case 1:
      while(1){
        Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        Analogue5 = analogRead(LEFT_BACK_LIR);
        Analogue6 = analogRead(LEFT_BACK_MIR);
        Analogue7 = analogRead(RIGHT_BACK_MIR);
        
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
      Lipo_level_cal = (raw_lipo - 717);
      Lipo_level_cal = Lipo_level_cal * 100;
      Lipo_level_cal = Lipo_level_cal / 143;
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
        distance4 = Kalman(temp, Distance4_Past, process_noise4, sensor_noise4, last_var4, 4);
        Serial.println(distance4);
        Distance4_Past = distance4;
        delay(25);
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
        Analogue6 = analogRead(LEFT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        temp = Distance(6, Analogue6);
        Serial.print(temp);
        Serial.print(" , ");
        distance6 = Kalman(temp, Distance6_Past, process_noise6, sensor_noise6, last_var6, 6);
        Serial.println(distance6);
        Distance6_Past = distance6;
        delay(25);

      }
    break;

    //USE MODE 7 TO PRINT OUT THE RAW AND FILTERED READINGS FOR THE BACK RIGHT MIR SENSOR
    case 7:
      while(1){
        Analogue7 = analogRead(RIGHT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        temp = Distance(7, Analogue7);
        Serial.print(temp);
        Serial.print(" , ");
        distance7 = Kalman(temp, Distance7_Past, process_noise7, sensor_noise7, last_var7 ,7);
        Serial.println(distance7);
        Distance7_Past = distance7;
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
        Analogue6 = analogRead(LEFT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        Analogue7 = analogRead(RIGHT_BACK_MIR); // the read out is a signal from 0-1023 corresponding to 0-5v

        temp = Distance(6, Analogue6);
        distance6 = Kalman(temp, Distance6_Past, process_noise6, sensor_noise6, last_var6, 6);
        Distance6_Past = distance6;

        Serial.print(distance6);
        Serial.print(",");

        temp = Distance(7, Analogue7);
        distance7 = Kalman(temp, Distance7_Past, process_noise7, sensor_noise7, last_var7, 7);
        Serial.println(distance7);
        Distance7_Past = distance7;
        delay(25);
      }
    break;
  }
}

double Distance(int code, int RawADC) { // find distances using calibration curve equations
  double dist;
  
  switch(code) {
    case 1 ... 4:
      dist = 3127.4*pow(RawADC, -.92);//Callibration for Left Front
//      Serial.println(dist);
//      dist = 4E-14*pow(RawADC,6) -8E-11*pow(RawADC,5) + 7E-08*pow(RawADC,4) - 3E-05*pow(RawADC,3) + 0.0085*pow(RawADC,2) - 1.262*RawADC + 108.47;
    break;
    case 5:
      dist = 1790.5*pow(RawADC, -.829);//Callibration for Left Back
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
