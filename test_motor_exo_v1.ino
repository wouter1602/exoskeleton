#include <Arduino.h>

// Motor PWM value
#define MOTOR_1_PWM_PIN 9
#define MOTOR_2_PWM_PIN 10

// Motor rotation
#define MOTOR_1_LEFT_EN 4
#define MOTOR_2_LEFT_EN 5

#define MOTOR_1_RIGHT_EN 2
#define MOTOR_2_RIGHT_EN 3

// Potentiometer
#define POTENT_1 4
#define POTENT_2 3



int rotDirection = 0;

/**
 * Motor struct with all the values from an single motor combined.
 */
struct Motor {
  uint8_t pwm_pin = 0;
  uint8_t pwm_value = 0;
  uint8_t enable_rotate_left_pin = 0;
  uint8_t enable_rotate_right_pin = 0;
  bool rot_direction = true;            // True is rotating clockwise, false is counter clockwise.
  uint8_t potentiometer = 0;
  uint16_t ADC_value = 0;
};

/**
 * Function sets the inputs and outputs used by the motor driver
 * @param motor Motor struct with the corresponding pins
 * @return void
 */
void setup_motor(const Motor &motor) {
    pinMode(motor.pwm_pin, OUTPUT);
    pinMode(motor.enable_rotate_left_pin, OUTPUT);
    pinMode(motor.enable_rotate_right_pin, OUTPUT);
    pinMode(motor.potentiometer, INPUT);

    // Initial direction
    digitalWrite(motor.enable_rotate_left_pin, LOW);
    digitalWrite(motor.enable_rotate_right_pin, HIGH);
}

/**
 * Set up the i/o of the Arduino.
 * @return void
 */
void setup() {
  Motor upper_motor;
  upper_motor.pwm_pin = MOTOR_1_PWM_PIN;                    //PWM pin
  upper_motor.ADC_value = 50;                               // Default motor speed
  upper_motor.enable_rotate_left_pin = MOTOR_1_LEFT_EN;     // Enable left rotation pin
  upper_motor.enable_rotate_right_pin = MOTOR_1_RIGHT_EN;   // Enable right rotation pin
  upper_motor.potentiometer = POTENT_1;                     // Potentiometer ADC pin

  Motor lower_motor;
  upper_motor.pwm_pin = MOTOR_2_PWM_PIN;                    //PWM pin
  upper_motor.ADC_value = 50;                               // Default motor speed
  upper_motor.enable_rotate_left_pin = MOTOR_2_LEFT_EN;     // Enable left rotation pin
  upper_motor.enable_rotate_right_pin = MOTOR_2_RIGHT_EN;   // Enable right rotation pin
  upper_motor.potentiometer = POTENT_2;                     // Potentiometer ADC pin

  setup_motor(upper_motor);                                 // Set input output for upper motor
  setup_motor(lower_motor);                                 // Set input output for lower motor

  Serial.begin(115200);
}

void loop() {
  int potValue = analogRead(A4); // Read potentiometer value
  Serial.println(potValue);
  //int pwmOutput = map(potValue, 0, 1023, 0 , 255); // Map the potentiometer value from 0 to 255
  int pwmOutput=50; // 0 is laagst 255 is hoogst
  //analogWrite(enA, pwmOutput); // Send PWM signal to L298N Enable pin
  // digitalWrite(enA,pwmOutput);
  // If button is pressed - change rotation direction
  if ((potValue<512)  && (rotDirection == 0)) {
    // digitalWrite(in1, HIGH);
    // digitalWrite(in2, LOW);
    Serial.println("naar voor");
    delay(10);
  }
  // If button is pressed - change rotation direction
  if ((potValue>512) && (rotDirection == 1)) {
    // digitalWrite(in1, LOW);
    // digitalWrite(in2, HIGH);
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
