// Motor PWM value
#define enA 9

// Motor rotation
#define in1 4
#define in2 2

// Potentiometer
#define POTENT_1 4

struct Motor
{
  uint8_t pwm_pin = 0;
  uint8_t pwm_value = 0;
  uint8_t enable_rotate_left_pin = 0;
  uint8_t enable_rotate_right_pin = 0;
  uint8_t potentiometer = 0;
  uint16_t ADC_value = 0;
};


int rotDirection = 0;
void setup() {
  Motor upper_motor;

  upper_motor.pwm_pin = 9;
  upper_motor.enable_rotate_left_pin = 4;
  upper_motor.enable_rotate_right_pin = 2;
  upper_motor.potentiometer = 4;

  Motor lower_motor;

  pinMode(upper_motor.pwm_pin, OUTPUT);
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
