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
int distanceLFL = 0; //for the calculated distance using the sensor callibrations
int distanceLBL = 0;
int distanceLBM = 0;
int distanceRBM = 0;

//----Kalman Filter----
double last_est_left = 0;
double last_est_right = 0;
double last_var = 999;
double process_noise = 1;
double sensor_noise = 10;    // Change the value of sensor noise to get different KF performance
//----Kalman Filter----

void setup() {
Serial.begin(9600); // start serial communication
}
void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Do you want to enter Callibration Mode [1] or Tuning Mode [2]?:");
  
  while (Serial.available() ==0){
  }
  int Mode = Serial.parseInt();

  while(Serial.readString() != "stop"){
      Analogue4 = analogRead(LEFT_FRONT_LIR); // the read out is a signal from 0-1023 corresponding to 0-5v
      Analogue5 = analogRead(LEFT_BACK_LIR);
      Analogue6 = analogRead(LEFT_BACK_MIR);
      Analogue7 = analogRead(RIGHT_BACK_MIR);
    switch (Mode){
    case 1:
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
      break;
    case 2:
      //Testing code goes here
    break;
    }
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

/* Kalman Filter for IR sensors
double Kalman(double rawdata, double prev_est){   // Kalman Filter
  double a_priori_est, a_post_est, a_priori_var, a_post_var, kalman_gain;

  a_priori_est = prev_est;  
  a_priori_var = last_var + process_noise; 

  kalman_gain = a_priori_var/(a_priori_var+sensor_noise);
  a_post_est = a_priori_est + kalman_gain*(rawdata-a_priori_est);
  a_post_var = (1- kalman_gain)*a_priori_var;
  last_var = a_post_var;
  return a_post_est;
}

double IR_filtered(IR code) { // find distances using calibration curve equations
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
      
  return est;
}*/
