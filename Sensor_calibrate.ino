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
//int distanceLFL = 0; //for the calculated distance using the sensor callibrations
//int distanceLBL = 0;
//int distanceLBM = 0;
//int distanceRBM = 0;

//----Kalman Filter----
double Filtered4 = 0;
double Filtered5 = 0;
double Filtered6 = 0;
double Filtered7 = 0;
double Filtered4_Past = 0;
double Filtered5_Past = 0;
double Filtered6_Past = 0;
double Filtered7_Past = 0;
double last_var4 = 9999;
double last_var5 = 9999;
double last_var6 = 9999;
double last_var7 = 9999;
double process_noise4 = 0.5;
double process_noise5 = 2;
double process_noise6 = 2;
double process_noise7 = 2;
double sensor_noise4 = 9;
double sensor_noise5 = 7;
double sensor_noise6 = 7;
double sensor_noise7 = 7;// Change the value of sensor noise to get different KF performance
//----Kalman Filter----


//Battery Check Variables
int Lipo_level_cal;
int raw_lipo;

void setup() {
Serial.begin(115200); // start serial communication
}
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Please enter the mode you want to run: Callibration Mode [1], Battery Test [2] or Enter the Sensor pin for the Sensor you want to tune [4 - 7]?:");
  
  while (Serial.available() ==0){
  }
  int Mode = Serial.parseInt();

  switch (Mode){
    case 1:
      while(Serial.available() !=0){
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
    case 2 ... 3:
    while(Serial.available() !=0){
      raw_lipo=analogRead(A0);
      //Serial.println(raw_lipo);
      Lipo_level_cal = (raw_lipo - 717);
      Lipo_level_cal = Lipo_level_cal * 100;
      Lipo_level_cal = Lipo_level_cal / 143;
      Serial.print("Lipo Level: ");
      Serial.print(Lipo_level_cal);
      Serial.println("%");
      delay(400);
    }
    break;
    case 4:
      while(1){
        Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
        //Testing code goes here
        Filtered4 = Kalman(Analogue4, Filtered4_Past, process_noise4, sensor_noise4, last_var4);
        
        Serial.print(0);
        Serial.print(",");
        Serial.print(Analogue4);
        Serial.print(",");
        Serial.print(Filtered4);
        Serial.print(",");
        Serial.println(650);
      }
    break;
    case 5:
      while(1){
        Analogue5 = analogRead(LEFT_BACK_LIR);
  
        //Testing code goes here
        Filtered5 = Kalman(Analogue5, Filtered5_Past, process_noise5, sensor_noise5, last_var5);
        
        Serial.print(0);
        Serial.print(",");
        Serial.print(Analogue5);
        Serial.print(",");
        Serial.print(Filtered5);
        Serial.print(",");
        Serial.println(650);
      }
    break;
    case 6:
      while(1){
        Analogue6 = analogRead(LEFT_BACK_MIR);
  
        //Testing code goes here
        Filtered6 = Kalman(Analogue6, Filtered6_Past, process_noise6, sensor_noise6, last_var6);
        
        Serial.print(0);
        Serial.print(",");
        Serial.print(Analogue6);
        Serial.print(",");
        Serial.print(Filtered6);
        Serial.print(",");
        Serial.println(650);
      }
    break;
    case 7:
      while(1){
        Analogue7 = analogRead(RIGHT_BACK_MIR);
  
        //Testing code goes here
        Filtered7 = Kalman(Analogue7, Filtered7_Past, process_noise7, sensor_noise7, last_var7);
        
        Serial.print(0);
        Serial.print(",");
        Serial.print(Analogue7);
        Serial.print(",");
        Serial.print(Filtered7);
        Serial.print(",");
        Serial.println(650);
      }
    break;
    }
   
  //int distanceLFL = 17948*pow(signalADC,-1.22); // calculate the distance using the datasheet graph
  //int distanceLBL = 46161*pow(signalADC,-1.302); // calculate the distance using the calibrated graph
  //int distanceLBM = 17948*pow(signalADC,-1.22); 
  //int distanceRBM = 46161*pow(signalADC,-1.302);

 /*Serial.print("distance datasheet graph "); //print out the results on serial monitor
  Serial.print(distance1);
  Serial.println("cm");
  Serial.print(" ");
  Serial.print("distance calibration graph ");
  Serial.print(distancec);
  Serial.println("cm");
  //Serial.print(0);
  //Serial.print("");
  //Serial.print(1000);
  //Serial.print("");*/
}

//Kalman Filter for IR sensors
double Kalman(double rawdata, double prev_est, double process_noise, double sensor_noise, double last_var){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}

/*double IR_filtered(IR code) { // find distances using calibration curve equations
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
  // random link https://www.norwegiancreations.com/2016/01/tutorial-multiple-values-in-the-arduino-ide-serial-plotter/    
  return est;
}*/ 
