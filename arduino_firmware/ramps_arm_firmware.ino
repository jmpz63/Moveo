// Arduino Mega RAMPS 1.4 Serial Firmware for Robotic Arm Control
// This sketch subscribes to joint commands via serial and publishes joint states via serial.

#include <AccelStepper.h> // For controlling stepper motors

// --- Serial Communication Configuration ---
#define SERIAL_BAUDRATE 115200 // High baud rate for faster communication

// --- Stepper Motor Configuration (for RAMPS 1.4) ---
// Define pins for each stepper motor.
// These are common RAMPS 1.4 pin assignments for X, Y, Z, E0, E1 axes.
// Adjust these if your wiring or RAMPS version differs.
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38 // Active LOW
#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56 // Active LOW
#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62 // Active LOW
#define E0_STEP_PIN        26
#define E0_DIR_PIN         28
#define E0_ENABLE_PIN      24 // Active LOW
#define E1_STEP_PIN        36
#define E1_DIR_PIN         34
#define E1_ENABLE_PIN      30 // Active LOW

// AccelStepper objects for each joint.
// We'll assume a 5-DOF arm, so 5 steppers.
// Mode 1: Driver takes care of step/dir pins.
AccelStepper stepper0(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN); // Joint 0 (e.g., Base)
AccelStepper stepper1(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN); // Joint 1 (e.g., Shoulder)
AccelStepper stepper2(AccelStepper::DRIVER, Z_STEP_PIN, Z_DIR_PIN); // Joint 2 (e.g., Elbow)
AccelStepper stepper3(AccelStepper::DRIVER, E0_STEP_PIN, E0_DIR_PIN); // Joint 3 (e.g., Wrist 1)
AccelStepper stepper4(AccelStepper::DRIVER, E1_STEP_PIN, E1_DIR_PIN); // Joint 4 (e.g., Wrist 2)

// Array to hold all steppers for easier iteration
AccelStepper* steppers[] = {&stepper0, &stepper1, &stepper2, &stepper3, &stepper4};
const int NUM_JOINTS = sizeof(steppers) / sizeof(steppers[0]);

// Enable pins array (active LOW)
const int enablePins[] = {X_ENABLE_PIN, Y_ENABLE_PIN, Z_ENABLE_PIN, E0_ENABLE_PIN, E1_ENABLE_PIN};

// --- Joint State Variables ---
// These arrays will store the current and target positions for each joint.
// Positions are in steps.
long current_joint_steps[NUM_JOINTS];
long target_joint_steps[NUM_JOINTS];

// --- Serial Communication Buffers ---
const int MAX_SERIAL_BUFFER = 128;
char serial_buffer[MAX_SERIAL_BUFFER];
int serial_buffer_idx = 0;

unsigned long last_state_publish_time = 0;
const unsigned long STATE_PUBLISH_INTERVAL_MS = 50; // Publish states every 50ms (20 Hz)

// --- Setup Function ---
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(10); // Set a short timeout for Serial.readStringUntil()

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // Turn off LED initially

  // Configure AccelStepper motors
  for (int i = 0; i < NUM_JOINTS; ++i) {
    steppers[i]->setMaxSpeed(2000.0);    // Adjust max speed (steps/sec)
    steppers[i]->setAcceleration(1000.0); // Adjust acceleration (steps/sec^2)
    pinMode(enablePins[i], OUTPUT);
    digitalWrite(enablePins[i], LOW); // Enable motors (active LOW)
    current_joint_steps[i] = 0; // Initialize current position
    target_joint_steps[i] = 0;  // Initialize target position
    steppers[i]->setCurrentPosition(0); // Set initial position
  }

  digitalWrite(LED_BUILTIN, HIGH); // Indicate setup complete
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
}

// --- Loop Function ---
void loop() {
  // Read incoming serial commands
  while (Serial.available()) {
    char inChar = Serial.read();
    if (inChar == '\n') { // End of command
      serial_buffer[serial_buffer_idx] = '\0'; // Null-terminate the string
      parseCommand(serial_buffer);
      serial_buffer_idx = 0; // Reset buffer index
    } else if (serial_buffer_idx < MAX_SERIAL_BUFFER - 1) {
      serial_buffer[serial_buffer_idx++] = inChar;
    }
  }

  // Update AccelStepper motors
  for (int i = 0; i < NUM_JOINTS; ++i) {
    steppers[i]->run(); // Must be called repeatedly to move the motor
    current_joint_steps[i] = steppers[i]->currentPosition();
  }

  // Publish joint states periodically
  if (millis() - last_state_publish_time >= STATE_PUBLISH_INTERVAL_MS) {
    publishJointStates();
    last_state_publish_time = millis();
  }
}

// --- Parse Incoming Serial Command ---
// Expected format: "J<joint_idx>:<target_step_value>"
void parseCommand(char* command) {
  if (command[0] == 'J') {
    int joint_idx = -1;
    long target_steps = 0;

    // Parse joint index
    char* colon_ptr = strchr(command, ':');
    if (colon_ptr != NULL) {
      *colon_ptr = '\0'; // Temporarily null-terminate to parse index
      joint_idx = atoi(&command[1]); // Convert char after 'J' to int
      *colon_ptr = ':'; // Restore colon

      // Parse target steps
      target_steps = atol(colon_ptr + 1); // Convert string after colon to long
    }

    if (joint_idx >= 0 && joint_idx < NUM_JOINTS) {
      target_joint_steps[joint_idx] = target_steps;
      steppers[joint_idx]->moveTo(target_joint_steps[joint_idx]);
      // Serial.print("Moving Joint "); Serial.print(joint_idx); Serial.print(" to "); Serial.println(target_steps);
    } else {
      Serial.print("Invalid joint command: "); Serial.println(command);
    }
  } else {
    Serial.print("Unknown command: "); Serial.println(command);
  }
}

// --- Publish Joint States via Serial ---
// Format: "S<joint0_steps>,<joint1_steps>,<joint2_steps>,<joint3_steps>,<joint4_steps>"
void publishJointStates() {
  Serial.print("S");
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial.print(current_joint_steps[i]);
    if (i < NUM_JOINTS - 1) {
      Serial.print(",");
    }
  }
  Serial.println(); // Newline to indicate end of message
}
