#define enA 9
#define in1 4
#define in2 2
int rotDirection = 0;
void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}
void loop() {
  int potValue = analogRead(A4); // Read potentiometer value
  Serial.println(potValue);
  //int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  int pwmOutput=50; // 0 is laagst 255 is hoogst
  //analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  digitalWrite(enA,pwmOutput);
  // If button is pressed - change rotation direction
  if ((potValue<512)  && (rotDirection == 0)) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("naar voor");
    delay(10);
  }
  // If button is pressed - change rotation direction
  if ((potValue>512) && (rotDirection == 1)) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println("naar achter");
      delay(10);
    }
    if(potValue<100){
      rotDirection=0;
  }
  if(potValue>900){
    rotDirection = 1;
  }
}
