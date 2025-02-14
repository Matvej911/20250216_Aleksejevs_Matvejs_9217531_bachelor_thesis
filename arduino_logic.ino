#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo myservo0;  // Pump
Servo myservo1;  // Solenoid

#define SERVOMIN  224  // Adjusted minimum pulse length
#define SERVOMAX  503  // Adjusted maximum pulse length
#define NUM_SERVOS 4  

#define ENA_PIN 12  // Pin for ENA (Enable)
#define DIR_PIN 8   // Pin for DIR (Direction)
#define PUL_PIN 3   // Pin for PUL (Pulse)

#define RELAY_PIN 2 // solenoid for pneumatic

#define STEP_DELAY 150

bool taskCompleted = false;  // Flag to indicate task completion


float initialAngles[NUM_SERVOS] = { 0, -25, -67, 89 };
float targetAngles[NUM_SERVOS];  // Will be received from Raspberry Pi
float PlaceAngles[NUM_SERVOS] = {65, 60, -54, 27 };
float ENDAngles[NUM_SERVOS] = {-65, 30, -54, 0 };


void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Ready to Receive Angles...");

  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);

  // Attach suction cup components
  myservo0.attach(7);  // Pump
  myservo1.attach(4);  // Solenoid

  // Set initial state
  myservo0.write(0);   // Pump OFF
  myservo1.write(0);   // Solenoid closed

  // Move all servos to initial positions
  moveAllServosToInitial(3);

  pinMode(ENA_PIN, OUTPUT);  // Ena output
  pinMode(DIR_PIN, OUTPUT);  // Dir output
  pinMode(PUL_PIN, OUTPUT);  // Pul output 

  digitalWrite(ENA_PIN, LOW);  // 

  pinMode(RELAY_PIN, OUTPUT);  // relay output pin 
  digitalWrite(RELAY_PIN, HIGH);  // the relay initialy closed 

  
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');  // Read until newline
    parseAngles(data);  // Convert received data to target angles

    Serial.println("Received New Angles:");
    for (int i = 0; i < NUM_SERVOS; i++) {
      Serial.print("Servo "); Serial.print(i); 
      Serial.print(": "); Serial.println(targetAngles[i]);
    }


    delay(3000);  
    moveSingleServoSmooth(3, targetAngles[3], 3);

    moveAllServosSmooth(targetAngles, 3);

    activateSuction();

    moveAllServosToHomeFromTarget(2);

    deactivateSuction();
   
    moveStepperLeft(15000, STEP_DELAY);

    moveSingleToPlace(10);

    moveAllServosToPlace(10);

    deactivateSuction();

    moveAllServosToHome(30);

    moveStepperRight(15000, STEP_DELAY);

    controlPneumaticValve(false);

    delay(1500); 

    moveStepperLeft(15000, STEP_DELAY);

    moveSingleToPlace(10);

    moveAllServosToPlace(10);

    activateSuction();

    moveAllServosToHome(30);

    moveAllServosEnd(ENDAngles, 30);

    deactivateSuction();

    moveStepperRight(15000, STEP_DELAY);
    
    moveAllServosToInitial(20);

    Serial.println("DONE");
    delay(3000);  


  }
}

// Parses received string and updates targetAngles
void parseAngles(String data) {
  Serial.print("Raw Data Received: ");
  Serial.println(data); 

  int index = 0;
  char *ptr = strtok((char*)data.c_str(), ",");
  while (ptr != NULL && index < NUM_SERVOS) {
    targetAngles[index] = atof(ptr);  

    // Debugging print
    Serial.print("Parsed Angle ");
    Serial.print(index);
    Serial.print(": ");
    Serial.println(targetAngles[index]);

    ptr = strtok(NULL, ",");
    index++;
  }

  // Send the parsed angles back to the Raspberry Pi
  Serial.print("Sending Target Angles: ");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.print(targetAngles[i]);
    Serial.print(",");
  }
  Serial.println(); 
}


void moveAllServosToInitial(int stepDelay) {
  Serial.println("Moving servos to initial positions...");
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, initialAngles[i]);
    delay(stepDelay);
  }
}

void moveSingleToPlace(int stepDelay) {
  Serial.println("Moving last servo to its home position...");
  
  int lastServoIndex = NUM_SERVOS - 1;  
  float targetAngle = PlaceAngles[lastServoIndex];
  float currentAngle = initialAngles[lastServoIndex];

  // Gradually move the last servo to the target angle
  while (abs(currentAngle - targetAngle) > 0.5) {
    currentAngle += (targetAngle > currentAngle) ? 0.5 : -0.5;  
    moveServo(lastServoIndex, currentAngle);  
    delay(stepDelay);  
  }
  moveServo(lastServoIndex, targetAngle);  
  delay(stepDelay);  // Small delay after moving the last servo
}


void moveAllServosToHome(int stepDelay) {
  Serial.println("Moving all servos to their home positions...");

  float currentAngles[NUM_SERVOS];
  
  // Initialize the current angles from the initial positions
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = PlaceAngles[i];
  }

  bool moving = true;
  
  while (abs(currentAngles[1] - initialAngles[1]) > 0.5) {
    currentAngles[1] += (initialAngles[1] > currentAngles[1]) ? 0.5 : -0.5; 
    moveServo(1, currentAngles[1]);  
    delay(stepDelay);  
  }

  while (moving) {
    moving = false;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
      if (i != 1) {  
        float targetAngle = initialAngles[i];  
        
        // Gradually move each servo to the target angle
        if (abs(currentAngles[i] - targetAngle) > 0.5) {
          moving = true;
          currentAngles[i] += (targetAngle > currentAngles[i]) ? 0.5 : -0.5;  
          moveServo(i, currentAngles[i]);  
        }
      }
    }
    
    delay(stepDelay); 
  }
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, initialAngles[i]);
  }
}


void moveAllServosToHomeFromTarget(int stepDelay) {
  Serial.println("Moving all servos to their home positions...");
  float currentAngles[NUM_SERVOS];
  
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = targetAngles[i];
  }

  bool moving = true;
  while (moving) {
    moving = false;
    
    // Check if any servo needs to move
    for (int i = 0; i < NUM_SERVOS; i++) {
      float targetAngle = initialAngles[i];  
      
      if (abs(currentAngles[i] - targetAngle) > 0.5) {
        moving = true;
        currentAngles[i] += (targetAngle > currentAngles[i]) ? 0.5 : -0.5;  
        moveServo(i, currentAngles[i]);  
      }
    }
    
    delay(stepDelay);  
  }
  

  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, initialAngles[i]);
  }
}

void moveAllServosToPlace(int stepDelay) {
  Serial.println("Moving all servos to their home positions...");


  float currentAngles[NUM_SERVOS];
  

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = initialAngles[i];
  }

  bool moving = true;
  while (moving) {
    moving = false;
    
   
    for (int i = 0; i < NUM_SERVOS; i++) {
      float targetAngle = PlaceAngles[i];  
      
      // Gradually move each servo to the target angle
      if (abs(currentAngles[i] - targetAngle) > 0.5) {
        moving = true;
        currentAngles[i] += (targetAngle > currentAngles[i]) ? 0.5 : -0.5; 
        moveServo(i, currentAngles[i]);  
      }
    }
    
    delay(stepDelay);  
  }
  
  // Ensure final position is set
  for (int i = 0; i < NUM_SERVOS; i++) {
    moveServo(i, PlaceAngles[i]);
  }
}

void moveAllServosEnd(float ENDAngles[], int stepDelay) {
  bool moving = true;
  float currentAngles[NUM_SERVOS];

  // Initialize current angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = initialAngles[i];
  }

  while (moving) {
    moving = false;
    for (int i = 0; i < 3; i++) {  
      if (abs(currentAngles[i] - ENDAngles[i]) > 0.5) {
        moving = true;
        currentAngles[i] += (ENDAngles[i] > currentAngles[i]) ? 0.5 : -0.5;
        moveServo(i, currentAngles[i]);
      }
    }
    delay(stepDelay);
  }
}

void moveSingleServoSmooth(int servoIndex, float targetAngle, int stepDelay) {
  float currentAngle = initialAngles[servoIndex];

  while (abs(currentAngle - targetAngle) > 0.5) {
    currentAngle += (targetAngle > currentAngle) ? 0.5 : -0.5;
    moveServo(servoIndex, currentAngle);
    delay(stepDelay);
  }
}

void moveAllServosSmooth(float targetAngles[], int stepDelay) {
  bool moving = true;
  float currentAngles[NUM_SERVOS];

  // Initialize current angles
  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngles[i] = initialAngles[i];
  }

  while (moving) {
    moving = false;
    for (int i = 0; i < 3; i++) {  
      if (abs(currentAngles[i] - targetAngles[i]) > 0.5) {
        moving = true;
        currentAngles[i] += (targetAngles[i] > currentAngles[i]) ? 0.5 : -0.5;
        moveServo(i, currentAngles[i]);
      }
    }
    delay(stepDelay);
  }
}

void moveServo(int servoNum, float angle) {
  int pulse = angleToPWM(angle);
  pwm.setPWM(servoNum, 0, pulse);
  Serial.print("Servo ");
  Serial.print(servoNum);
  Serial.print(" moved to: ");
  Serial.println(angle);
}

int angleToPWM(float angle) {
  int normalizedAngle = map(angle, -90, 90, 0, 180);
  return map(normalizedAngle, 0, 180, SERVOMIN, SERVOMAX);
}

// Activates the suction cup (pump ON, solenoid closed)
void activateSuction() {
  Serial.println("Activating suction...");
  myservo0.write(180);  // Turn pump ON
  myservo1.write(0); 
  delay(1000); 
  myservo0.write(0);
  delay(800);
}

// Deactivates the suction cup (pump OFF, solenoid closed)
void deactivateSuction() {
  Serial.println("Deactivating suction...");
  delay(1000);
  myservo1.write(180);  
  delay(1000);
}

void moveStepperLeft(int steps, int stepDelay) {
  digitalWrite(DIR_PIN, LOW);  // Set motor direction to left

  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

void moveStepperRight(int steps, int stepDelay) {
  digitalWrite(DIR_PIN, HIGH);  // Set motor direction to right

  for (int i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(stepDelay);
  }
}

// Function to control the pneumatic valve
void controlPneumaticValve(bool activate) {
  if (activate) {
    digitalWrite(RELAY_PIN, HIGH);  
    delay(1000); 
    digitalWrite(RELAY_PIN, LOW); 
  } else {
    digitalWrite(RELAY_PIN, LOW);  
    delay(1000); 
    digitalWrite(RELAY_PIN, HIGH); 
  }
}

