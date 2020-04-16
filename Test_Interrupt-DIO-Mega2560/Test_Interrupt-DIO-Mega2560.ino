#define EMG_BUTTON  2
#define EMG_LED 3

void setup(){
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(EMG_BUTTON, INPUT_PULLUP);
  pinMode(EMG_LED, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(EMG_BUTTON), executeEmgButton, LOW);
  // any interrupt mode (LOW, CHANGE, RISING, FALLING, HIGH)
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println(digitalRead(EMG_BUTTON));
  digitalWrite(EMG_LED, LOW);
}

void executeEmgButton(){
  int R = digitalRead(EMG_BUTTON);
  if(R == LOW){
    digitalWrite(EMG_LED, HIGH);
  }
}
