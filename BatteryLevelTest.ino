int Lipo_level_cal;
int raw_lipo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}



void loop() {
  // put your main code here, to run repeatedly:
raw_lipo=analogRead(A0);
Lipo_level_cal = (raw_lipo - 717);
Lipo_level_cal = Lipo_level_cal * 100;
Lipo_level_cal = Lipo_level_cal / 143;
Serial.print("Lipo Level: ");
Serial.print(Lipo_level_cal);
Serial.println("%");
//Serial.println(raw_lipo);
delay(500);

}
