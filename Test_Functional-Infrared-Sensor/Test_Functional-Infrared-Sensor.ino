void setup() {
  // initialization baud rate serial communication 
  Serial.begin(9600);
  // define particulary pin used as an input data sensor
  pinMode(2,INPUT);
}

void loop() {
  // read teh input pin:
  int readDataSensor = digitalRead(2);
  // print out reading of Infrared Sensor:
  Serial.println(readDataSensor);
  delay(1);
}
