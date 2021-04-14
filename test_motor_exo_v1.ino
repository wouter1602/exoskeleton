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
#define POTENT_1 A4
#define POTENT_2 A3

#define TARGET_OFFSET 20

/**
 * Motor struct with all the values from an single motor combined.
 */
struct Motor {
    uint8_t pwmPin = 0;
    uint8_t pwmValue = 160;
    uint8_t enableRotateLeftPin = 0;
    uint8_t enableRotateRightPin = 0;
    bool rot_direction = true;            // True is rotating clockwise, false is counter clockwise.
    uint8_t potentiometer = 0;
    uint16_t ADCValue = 0;
};

/**
 * Global defines of the Motor structs because otherwise they can't be used in setup() and loop().
 */
Motor upper_motor;
Motor lower_motor;

/**
 * @brief Setup Motor I/O
 * Function sets the inputs and outputs used by the motor driver
 * @param motor Motor struct with the corresponding pins
 */
void setupMotor(const Motor &motor) {
    pinMode(motor.pwmPin, OUTPUT);
    pinMode(motor.enableRotateLeftPin, OUTPUT);
    pinMode(motor.enableRotateRightPin, OUTPUT);
    pinMode(motor.potentiometer, INPUT);

    // Initial direction
    digitalWrite(motor.enableRotateLeftPin, LOW);
    digitalWrite(motor.enableRotateRightPin, HIGH);
}

/**
 * @brief Set the Motor Speed and orientation
 * If motor speed is set to 0 this function stops the motor
 * Otherwise it set the orientation and the give speed in the Motor struct
 * @param motor Motor struct with the pins, rotation and speed
 */
void setMotorSpeed(const Motor &motor) {
    if (motor.pwmValue == 0) {                          // Motor should not rotate
        analogWrite(motor.pwmPin, motor.pwmValue);      // Disable PWM signal
        digitalWrite(motor.enableRotateLeftPin, LOW);   // Disable left enable
        digitalWrite(motor.enableRotateRightPin, LOW);  // Disable right enable
        return;
    }
    if (motor.rot_direction) {                          // Rotate right
        digitalWrite(motor.enableRotateLeftPin, LOW);   // Disable left enable
        digitalWrite(motor.enableRotateRightPin, HIGH); // Enable right enable
    } else {                                            // Rotate left
        digitalWrite(motor.enableRotateLeftPin, HIGH);  // Enable left enable
        digitalWrite(motor.enableRotateRightPin, LOW);  // Disable right enable
    }

    analogWrite(motor.pwmPin, motor.pwmValue);          // Set motor speed
}

/**
 * @brief Read angle of the motor
 * Reads ADC value of potentiometer defined in the Motor struct.
 * ADC value is saved in the ADC_value in the struct.
 * @param motor Motor struct with the potentiometer and ADC value
 */
void readPotentiometer(Motor &motor) {
    motor.ADCValue = analogRead(motor.potentiometer);      // Read ADC value
}

/**
 * @brief Move motor to target angle
 * The function mesures the current angle the motor has.
 * It checks if the motor should rotate left or right and enables the motor with the given PWM speed.
 * If the motor angle is the same as the targetAngle within the offset the motor will be disabled.
 * @param motor Motor struct with the motor settings
 * @param targetAngle Angle the motor should move to
 * @param offset Mesurement offset
 * @param speed PWM speed the motor should move
 */
void goToTargetAngle(Motor &motor, uint16_t targetAngle, uint8_t offset, uint8_t speed) {
    readPotentiometer(motor);                              // Read Potentiometer
    if ((motor.ADCValue - offset) > targetAngle) {         // If true rotate clockwise
        motor.rot_direction = true;
        motor.pwmValue = speed;
        setMotorSpeed(motor);
    } else if ((motor.ADCValue + offset) < targetAngle) {  // If true rotate counter clockswise
        motor.rot_direction = false;
        motor.pwmValue = speed;
        setMotorSpeed(motor);
    } else {                                                // If true should stop rotating
        motor.pwmValue = 0;
        setMotorSpeed(motor);
    }
    
}

/**
 * @brief Setup the I/O of the Arduino
 * Setup motor values pin numbers and Serial for debuging.
 */
void setup() {
  upper_motor.pwmPin = MOTOR_1_PWM_PIN;                 // PWM pin
  upper_motor.pwmValue = 160;                           // Default motor speed
  upper_motor.enableRotateLeftPin = MOTOR_1_LEFT_EN;    // Enable left rotation pin
  upper_motor.enableRotateRightPin = MOTOR_1_RIGHT_EN;  // Enable right rotation pin
  upper_motor.potentiometer = POTENT_1;                 // Potentiometer ADC pin

  lower_motor.pwmPin = MOTOR_2_PWM_PIN;                 // PWM pin
  lower_motor.pwmValue = 160;                           // Default motor speed
  lower_motor.enableRotateLeftPin = MOTOR_2_LEFT_EN;    // Enable left rotation pin
  lower_motor.enableRotateRightPin = MOTOR_2_RIGHT_EN;  // Enable right rotation pin
  lower_motor.potentiometer = POTENT_2;                 // Potentiometer ADC pin

  setupMotor(upper_motor);                              // Set input output for upper motor
  setupMotor(lower_motor);                              // Set input output for lower motor

  Serial.begin(115200);
}

/**
 * @brief Arduino loop function
 * rotates the motors of the exoselton
 */
void loop() {
    readPotentiometer(upper_motor);                     // Read upper motor ADC value
    readPotentiometer(lower_motor);                     // Read lower motor ADC value

    Serial.print(upper_motor.ADCValue);                 // Print ADC values for debugging
    Serial.print(", ");
    Serial.println(lower_motor.ADCValue);

    // set_motor_speed(upper_motor);

    goToTargetAngle(upper_motor, 600, TARGET_OFFSET, 160);

    // int pwmOutput=50; // 0 is laagst 255 is hoogst
    // // If button is pressed - change rotation direction
    // if ((upper_motor.ADC_value < 512) && (upper_motor.rot_direction == false)) {
    //     Serial.println("Naar voor");
    //     delay(10);
    // }
  
    // // If button is pressed - change rotation direction
    // if ((upper_motor.ADC_value > 512) && (upper_motor.rot_direction == true)) {
    //     Serial.println("Naar achter");
    // }

    // if (upper_motor.ADC_value<450) {
    //     upper_motor.rot_direction = false;
    // }
    
    // if (upper_motor.ADC_value> 700) {
    //     upper_motor.rot_direction = true;
    // }
}
