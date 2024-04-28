// Define the number of steps per revolution for each stepper motor
#define STEPS_PER_REVOLUTION 200
// Define motor step delay (adjust as needed)
#define STEP_DELAY 500

#define Forward HIGH
#define Backward LOW
#define MAx_system_possible_steps 1000

//**********************************************************************************************************//
struct MotorControlPins {
  // Define the pins connected to each stepper motor driver
  int MOTOR1_STEP_PIN;
  int MOTOR1_DIR_PIN;
  int MOTOR2_STEP_PIN;
  int MOTOR2_DIR_PIN;
  int MOTOR3_STEP_PIN;
  int MOTOR3_DIR_PIN;
};
MotorControlPins motorPins = {2, 3, 4, 5, 6, 7};
//**********************************************************************************************************//
struct Motors {
  // no of motors
  uint8_t MOTOR1;
  uint8_t MOTOR2;
  uint8_t MOTOR3;
};
Motors motors = {1, 2, 3};
//**********************************************************************************************************//
struct MotorInterruptPins {
  // Define the pins connected to each stepper motor driver
  int Motor1_LimitSwitch;
  int Motor2_LimitSwitch;
  int Motor3_LimitSwitch;
};
MotorInterruptPins InterruptPins = {8, 9, 10};
//**********************************************************************************************************//
struct MotorLimitSwitch_Flags {
  bool Motor1_LimitSwitch_Flag;
  bool Motor2_LimitSwitch_Flag;
  bool Motor3_LimitSwitch_Flag;
};
MotorLimitSwitch_Flags LimitSwitch_Flags = {false, false, false};
//**********************************************************************************************************//
struct MotorPosition {
  // Initialize motor step counts and directions
  int Motor1;
  int motor2;
  int motor3;
};
MotorPosition currentPosition = {0, 0, 0};
//**********************************************************************************************************//

void setup() {
  Pins_setup();
  init_interrupt_pins();  // for limit switches
  // Initialize serial communication
  Serial.begin(9600);

  initialise();
}
//**********************************************************************************************************//

void loop() {
}

//**********************************************************************************************************//
// Function to move the specified stepper motor by one step
void moveMotor(int stepPin, int dirPin, bool Direction) {
  // Set direction (assuming clockwise)
  digitalWrite(dirPin, Direction);

  // Toggle step pin to move one step
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(STEP_DELAY);
}

//**********************************************************************************************************//
void moveMotorOneStep(uint8_t motorNumber, bool Direction) {

  // Determine pins based on motor
  switch (motorNumber) {
    case 1:
      moveMotor(motorPins.MOTOR1_STEP_PIN, motorPins.MOTOR1_DIR_PIN, Direction);
      break;
    case 2:
      moveMotor(motorPins.MOTOR2_STEP_PIN, motorPins.MOTOR2_DIR_PIN, Direction);
      break;
    case 3:
      moveMotor(motorPins.MOTOR3_STEP_PIN, motorPins.MOTOR3_DIR_PIN, Direction);
      break;
    default:
      return;  // Invalid motor
  }
}

//**********************************************************************************************************//
void Pins_setup() {
  // Set the motor pins as outputs
  pinMode(motorPins.MOTOR1_STEP_PIN, OUTPUT);
  pinMode(motorPins.MOTOR1_DIR_PIN, OUTPUT);
  pinMode(motorPins.MOTOR2_STEP_PIN, OUTPUT);
  pinMode(motorPins.MOTOR2_DIR_PIN, OUTPUT);
  pinMode(motorPins.MOTOR3_STEP_PIN, OUTPUT);
  pinMode(motorPins.MOTOR3_DIR_PIN, OUTPUT);
}

//**********************************************************************************************************//
void init_interrupt_pins() {
  pinMode(InterruptPins.Motor1_LimitSwitch, INPUT_PULLUP);
  pinMode(InterruptPins.Motor2_LimitSwitch, INPUT_PULLUP);
  pinMode(InterruptPins.Motor3_LimitSwitch, INPUT_PULLUP);
  delayMicroseconds(100000);  //100 ms
  attachInterrupt(digitalPinToInterrupt(InterruptPins.Motor1_LimitSwitch), []{limit_switch(motors.MOTOR1);}, RISING);
  attachInterrupt(digitalPinToInterrupt(InterruptPins.Motor2_LimitSwitch), []{limit_switch(motors.MOTOR2);}, RISING);
  attachInterrupt(digitalPinToInterrupt(InterruptPins.Motor3_LimitSwitch), []{limit_switch(motors.MOTOR3);}, RISING);
}

//**********************************************************************************************************//
void limit_switch(uint8_t motorNumber) {  //sets flag true
  switch (motorNumber) {
    case 1:
      digitalWrite(motorPins.MOTOR1_STEP_PIN, LOW);
      LimitSwitch_Flags.Motor1_LimitSwitch_Flag = true;
      break;
    case 2:
      digitalWrite(motorPins.MOTOR2_STEP_PIN, LOW);
      LimitSwitch_Flags.Motor2_LimitSwitch_Flag = true;
      break;
    case 3:
      digitalWrite(motorPins.MOTOR3_STEP_PIN, LOW);
      LimitSwitch_Flags.Motor3_LimitSwitch_Flag = true;
      break;
    default:
      Serial.println("Invalid motor limitswitch!");
      break;
  }
}

//**********************************************************************************************************//
uint8_t initialise() {
  int counter = 0;
  int zero_position_steps = 75;
  while (1) {  // initially moves until all the motor reaches its limit switches
    counter++;
    if (digitalRead(InterruptPins.Motor1_LimitSwitch) == LOW) {
      moveMotorOneStep(motors.MOTOR1, Backward);
    }
    if (digitalRead(InterruptPins.Motor2_LimitSwitch) == LOW) {
      moveMotorOneStep(motors.MOTOR2, Backward);
    }
    if (digitalRead(InterruptPins.Motor3_LimitSwitch) == LOW) {
      moveMotorOneStep(motors.MOTOR3, Backward);
    }

    if ((digitalRead(InterruptPins.Motor1_LimitSwitch) == HIGH) && (digitalRead(InterruptPins.Motor2_LimitSwitch) == HIGH) && (digitalRead(InterruptPins.Motor3_LimitSwitch) == HIGH)) {
      break;  // breaks while loop
    }

    if (counter == MAx_system_possible_steps) { // System moved maximum to its limits and limit switch has not detected
      Serial.println("System moving above its position limits, Limit switch not detected");
      Serial.println("Possible limit switch error");
      return 1;
    }

  }  // while loop breaks if all motor reaches its limit switch

  for (int i = 0; i < 100; i++) {  // moves 100 steps forward
    moveMotorOneStep(motors.MOTOR1, Forward);
    moveMotorOneStep(motors.MOTOR2, Forward);
    moveMotorOneStep(motors.MOTOR3, Forward);
  }

  for (int i = 0; i < zero_position_steps; i++) {  // moves 75 steps Backward, now pin states should be low else (adjust value = 75 according to system switch response)
    moveMotorOneStep(motors.MOTOR1, Backward);
    moveMotorOneStep(motors.MOTOR2, Backward);
    moveMotorOneStep(motors.MOTOR3, Backward);
  }

  if ((digitalRead(InterruptPins.Motor1_LimitSwitch) == LOW) && (digitalRead(InterruptPins.Motor2_LimitSwitch) == LOW) && (digitalRead(InterruptPins.Motor3_LimitSwitch) == LOW)) {
    Serial.println("Initialization success");
    // Setting positions to zero position after successful installatiion
    currentPosition.Motor1 = 0;
    currentPosition.motor2 = 0;
    currentPosition.motor3 = 0;
    return 0;
  }
  else{
    Serial.println("System Initialization Failed");
    Serial.println("Please adjust 'zero_position_steps' if necessary");
    return 2;
  }
}
