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
 * Global defines of the Motor structs because otherwise they can't be used in setup() and loop().
 */
Motor upper_motor;
Motor lower_motor;

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
 * If motor speed is set to 0 this function stops the motor
 * Otherwise it set the orientation and the give speed in the Motor struct
 * @param motor Motor struct with the pins, rotation and speed
 * @return void
 */
void set_motor_speed(const Motor &motor) {
    if (motor.pwm_value == 0) {                             // Motor should not rotate
        analogWrite(motor.pwm_pin, motor.pwm_value);        // Disable PWM signal
        digitalWrite(motor.enable_rotate_left_pin, LOW);    // Disable left enable
        digitalWrite(motor.enable_rotate_right_pin, LOW);   // Disable right enable
        return;
    }
    if (motor.rot_direction) {                              // Rotate right
        digitalWrite(motor.enable_rotate_left_pin, LOW);    // Disable left enable
        digitalWrite(motor.enable_rotate_right_pin, HIGH);  // Enable right enable
    } else {                                                // Rotate left
        digitalWrite(motor.enable_rotate_left_pin, HIGH);   // Enable left enable
        digitalWrite(motor.enable_rotate_right_pin, LOW);   // Disable right enable
    }

    analogWrite(motor.pwm_pin, motor.pwm_value);            // Set motor speed
}

/**
 * Reads ADC value of potentiometer defined in the Motor struct.
 * ADC value is saved in the ADC_value in the struct.
 * @param motor Motor struct with the potentiometer and ADC value
 * @return void 
 */
void read_potentiometer(Motor &motor) {
    motor.ADC_value = analogRead(motor.potentiometer);      // Read ADC value
}

/**
 * Set up the I/O of the Arduino.
 * @return void
 */
void setup() {
  upper_motor.pwm_pin = MOTOR_1_PWM_PIN;                    // PWM pin
  upper_motor.ADC_value = 50;                               // Default motor speed
  upper_motor.enable_rotate_left_pin = MOTOR_1_LEFT_EN;     // Enable left rotation pin
  upper_motor.enable_rotate_right_pin = MOTOR_1_RIGHT_EN;   // Enable right rotation pin
  upper_motor.potentiometer = POTENT_1;                     // Potentiometer ADC pin

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
    read_potentiometer(upper_motor);                        // Read upper motor ADC value
    read_potentiometer(lower_motor);                        // Read lower motor ADC value

    Serial.print(upper_motor.ADC_value);                    // Print ADC values for debugging
    Serial.print(", ");
    Serial.println(lower_motor.ADC_value);

    set_motor_speed(upper_motor);

    int pwmOutput=50; // 0 is laagst 255 is hoogst
    // If button is pressed - change rotation direction
    if ((upper_motor.ADC_value < 512) && (upper_motor.rot_direction == false)) {
        Serial.println("Naar voor");
        delay(10);
    }
  
    // If button is pressed - change rotation direction
    if ((upper_motor.ADC_value > 512) && (upper_motor.rot_direction == true)) {
        Serial.println("Naar achter");
    }

    if (upper_motor.ADC_value<100) {
        upper_motor.rot_direction = false;
    }
    
    if (upper_motor.ADC_value> 900) {
        upper_motor.rot_direction = true;
    }
}
