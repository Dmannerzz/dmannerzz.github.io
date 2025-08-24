// ========== Include Libraries ==========
#include <Wire.h>                   // I2C communication for LCD
#include <IRremote.h>              // IR remote control library
#include <Servo.h>                 // Servo motor control
#include <LiquidCrystal_I2C.h>     // I2C LCD library
#include <DHT.h>                   // DHT sensor library (temperature & humidity)

// ========== Pin Definitions ==========
#define IR_PIN 11                  // IR receiver input pin
#define TRIG_PIN 3                 // Ultrasonic sensor trigger pin
#define ECHO_PIN 4                 // Ultrasonic sensor echo pin
#define MOTOR_IN1 5                // Motor A input 1
#define MOTOR_IN2 6                // Motor A input 2
#define MOTOR_IN3 7                // Motor B input 1
#define MOTOR_IN4 8                // Motor B input 2
#define ENA 9                      // Motor A enable pin (PWM)
#define ENB 10                     // Motor B enable pin (PWM)
#define LED_PIN 2                 // Main LED pin
#define BUZZER_PIN 12             // Active buzzer pin
#define LDR_PIN A2                // Light-dependent resistor (LDR) analog pin
#define DHT_PIN A3                // DHT11 sensor pin
#define SERVO_PIN A0              // Servo control pin
#define I2C_ADDR 0x27             // I2C address for LCD

// ========== Constants ==========
#define DHTTYPE DHT11             // DHT sensor type
#define LCD_ADDRESS 0x27          // I2C LCD address
#define LCD_COLUMNS 16            // LCD width
#define LCD_ROWS 2                // LCD height

// ========== Objects ==========
Servo servo;                      // Servo motor object
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);  // LCD object
DHT dht(DHT_PIN, DHTTYPE);       // DHT11 sensor object

int currentSpeed = 150;          // Default motor speed

// ========== Data Structure ==========
struct RouteStep {
  uint8_t direction;             // Direction taken
  unsigned long duration;        // Duration of the step in milliseconds
};

// ========= Patrol Mode Variables =========
unsigned long duration = 0;
bool isPatrolRunning = false;            // Is patrol currently active?
unsigned long patrolStepStartTime = 0;   // Timestamp when current patrol step started
uint8_t currentPatrolStep = 0;           // Index of current patrol step
RouteStep* currentRoute;                 // Pointer to current route array
uint8_t currentRouteLength = 0;          // Length of current route
const unsigned long transitionDelay = 0; // Optional delay between steps
bool waitingForRouteConfirm = false; // Flag indicating if we're waiting for a second press to confirm patrol route source
unsigned long routeSelectStart = 0; // Stores the time when the first press occurred, to measure timeout window
bool obstacleInUse = false;


// ========== Global State Variables ==========
unsigned long lastActionTime = 0;        // Time of last user action
bool nightOverride = false;              // Force night mode
unsigned long buzzerTimer = 0;           // For timed buzzer events
bool buzzerState = false;                // Buzzer on/off
bool isIdle = false;                     // Is robot idle?
bool autoLEDOverride = false;            // Night LED override
bool autoObstacleLED = false;            // Auto LED in obstacle avoidance
int previousObstacleLED = 0;             // To restore LED mode after obstacle mode
int previousLedMode = 0;                 // To restore LED mode after an intruder is detected
int ledMode = 0;                         // LED mode selector (0–4)
unsigned long ledTimer = 0;              // Timer for LED effects
bool ledState = false;                   // LED state toggle
unsigned long currentMoveStart = 0;      // When did current movement begin?
bool irReady = false;                    // Has IR remote initialized?

// ========== Route Logging ==========
const uint8_t MAX_ROUTE_STEPS = 30;      // Max steps per route

RouteStep manualRoute[MAX_ROUTE_STEPS];   // Route recorded manually
RouteStep obstacleRoute[MAX_ROUTE_STEPS]; // Route recorded automatically

bool useManualRoute = true;               // Patrol mode route toggle

uint8_t manualRouteIndex = 0;             // Steps recorded in manualRoute
uint8_t obstacleRouteIndex = 0;           // Steps recorded in obstacleRoute

uint8_t autoRouteCount = 0;               // Total auto steps recorded
bool isLoggingManualRoute = false;        // Flag to log manual route
bool isLoggingObstacleRoute = false;      // Flag to log obstacle route

uint8_t lastDirection = 0;                // Last direction moved (for logging)

// ========== LED Flash State (Patrol Mode) ==========
int flashState = 0;                       // For LED/Buzzer animation sequencing
unsigned long lastChange = 0;            // Timing for LED flash change
int freq = 500;                           // Not currently used
int sweepDir = 1;                         // Not currently used

// ========== Obstacle Avoidance ==========
uint8_t lastAutoDirection = 255;          // Last direction moved in obstacle mode
unsigned long lastAutoMoveDuration = 0;   // How long did we move last
unsigned long autoMoveStartTime = 0;      // When did we start moving

// ========== Light-Based Night Mode ==========
bool isNightTime() {
  int lightLevel = analogRead(LDR_PIN);   // Read from LDR
  return lightLevel < 200;                // Threshold for night mode
}

// ========== Intruder Detection ==========
bool detectIntruder() {
  const int intruderThreshold = 10;       // Trigger distance in cm
  long duration;
  int distance;

  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30 ms timeout

  if (duration == 0) return false;

  distance = duration * 0.034 / 2;

  return distance > 0 && distance <= intruderThreshold;
}

// ========== Direction Definitions ==========
#define DIR_NONE -1
#define DIR_FORWARD 1
#define DIR_BACKWARD 2
#define DIR_LEFT 3
#define DIR_RIGHT 4
#define DIR_ROTATE_LEFT 5
#define DIR_ROTATE_RIGHT 6

const int SAFE_CLEARANCE_CM = 45;         // Obstacle avoidance clearance in cm

// ========== Mode Enum ==========
enum Mode { MANUAL, O_AVOIDANCE, RETURN_HOME, PATROL, IDLE, SLEEP };
Mode currentMode = IDLE;                  // Default state is idle


// ========== Functions ==========

void returnHome() {
  // Select the route to replay
  RouteStep* originalRoute;
  int originalLength;

  if (isLoggingManualRoute) {
    originalRoute = manualRoute;
    originalLength = manualRouteIndex;
  } else {
    originalRoute = obstacleRoute;
    originalLength = obstacleRouteIndex;
  }

  // Safety check on length
  if (originalLength > MAX_ROUTE_STEPS) originalLength = MAX_ROUTE_STEPS;

  // Reverse route as-is
  RouteStep reversedRoute[MAX_ROUTE_STEPS];
  for (int i = 0; i < originalLength; i++) {
    int reverseIndex = originalLength - 1 - i;
    reversedRoute[i] = originalRoute[reverseIndex]; // no direction change
  }

  // Replay reversed steps
  for (int i = 0; i < originalLength; i++) {
    RouteStep step = reversedRoute[i];

    switch (step.direction) {
      case DIR_FORWARD:  moveForward();  break;
      case DIR_BACKWARD: moveBackward(); break;
      case DIR_LEFT:     turnLeft();     break;
      case DIR_RIGHT:    turnRight();    break;
      default:           stopMoving();   break;
    }

    delay(step.duration);
    delay(500); // Optional buffer
  }
  stopMoving();
  // Final state or display
  // lcd.clear();
  // lcd.print("Home position reached");
  // delay(2000);
  // currentMode = IDLE;
}

void startPatrol() {
  isIdle = false;
  isPatrolRunning = true;
  currentPatrolStep = 0;

  currentRoute = useManualRoute ? manualRoute : obstacleRoute;
  currentRouteLength = useManualRoute ? manualRouteIndex : obstacleRouteIndex;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting Patrol:");
  lcd.setCursor(0, 1);
  lcd.print(useManualRoute ? "Manual route" : "Obstacle route");
  delay(1500);

  if (currentRouteLength == 0) {
    lcd.clear();
    lcd.print("No patrol route!");
    delay(2000);
    currentMode = IDLE;
    isPatrolRunning = false;
    return;
  }

  executeCurrentPatrolStep();
}
  

void welcomeAnimation() {
  for (int i = 0; i < 10; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}


void moveForward() {
  currentMoveStart = millis();
  lastDirection = 1;
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Moving forward..."));
  Serial.print("Moving forward");
  Serial.println(currentSpeed);

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  lastDirection = 1;
}

void moveBackward() {
  currentMoveStart = millis();
  lastDirection = 2;
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Moving backward..."));
  Serial.print("Moving backward");


  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);

  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void turnLeft() {
  currentMoveStart = millis();
  lastDirection = 3;
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Turning left..."));
  Serial.print("Turning left");

  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);
  digitalWrite(MOTOR_IN4, LOW);
  
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

}

void turnRight() {
  currentMoveStart = millis();
  lastDirection = 4;
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Turning right..."));
  Serial.print("Turning right...");

  digitalWrite(MOTOR_IN1, HIGH);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, HIGH);
  
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

}

void rotateInPlaceLeft() {
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Rotating CCW..."));

  digitalWrite(MOTOR_IN1, LOW);   // Left motor backward
  digitalWrite(MOTOR_IN2, HIGH);
  digitalWrite(MOTOR_IN3, HIGH);  // Right motor forward
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  digitalWrite(MOTOR_IN1, HIGH);  // Left motor forward
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);   // Right motor backward
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

  Serial.print("Rotating CCW");
}

void rotateInPlaceRight() {
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  lcd.setCursor(0, 1);
  lcd.print(F("Rotating CW..."));
  
  digitalWrite(MOTOR_IN1, HIGH);  // Left motor forward
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);   // Right motor backward
  digitalWrite(MOTOR_IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
  
  Serial.print("Rotating CW");
}
void logRouteStep(uint8_t dir, unsigned long duration, bool isManual) {
  if (isManual) {
    if (isLoggingManualRoute && manualRouteIndex < MAX_ROUTE_STEPS) {
      manualRoute[manualRouteIndex++] = { dir, duration };
      Serial.print("Logged Manual Step: ");
      Serial.print(dir);
      Serial.print(" | Duration: ");
      Serial.print(duration);
      Serial.print("ms | Index: ");
      Serial.println(manualRouteIndex);
    } else if (!isLoggingManualRoute) {
      Serial.println("Manual logging not active.");
    } else {
      Serial.println("Manual route memory full.");
    }
  } else {
    if (obstacleRouteIndex < MAX_ROUTE_STEPS) {
      if (obstacleRouteIndex > 0 && obstacleRoute[obstacleRouteIndex - 1].direction == dir) {
        // Extend duration of previous step
        obstacleRoute[obstacleRouteIndex - 1].duration += duration;
        Serial.print("Extended Auto Step: ");
        Serial.print(dir);
        Serial.print(" | +");
        Serial.print(duration);
        Serial.print("ms | New Duration: ");
        Serial.println(obstacleRoute[obstacleRouteIndex - 1].duration);
      } else {
        // New direction step
        obstacleRoute[obstacleRouteIndex++] = { dir, duration };
        Serial.print("Logged Auto Step: ");
        Serial.print(dir);
        Serial.print(" | Duration: ");
        Serial.print(duration);
        Serial.print("ms | Index: ");
        Serial.println(obstacleRouteIndex);
      }
      autoRouteCount = obstacleRouteIndex;
    } else {
      Serial.println("Obstacle route memory full.");
    }
  }
}


void stopMoving() {
  Serial.println("Stopped");
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  digitalWrite(MOTOR_IN3, LOW);
  digitalWrite(MOTOR_IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0); 

  unsigned long duration = millis() - currentMoveStart;
  if (currentMode == MANUAL && isLoggingManualRoute) {
    if (lastDirection != DIR_NONE) {
      if (currentMode == MANUAL && isLoggingManualRoute) {
        logRouteStep(lastDirection, duration, true);  // manual
      } else if (currentMode == O_AVOIDANCE) {
        logRouteStep(lastDirection, duration, false); // obstacle
      }
    } else {
      Serial.println("Skipping log: No valid direction before OK press.");
    }
  }
  
}

void executeCurrentPatrolStep() {
  Serial.print("Executing patrol steps");
  if (currentPatrolStep >= currentRouteLength) {
    stopMoving();
    lcd.clear();
    lcd.print("Patrol Done");
    currentMode = IDLE;
    isPatrolRunning = false;
    return;
  }

  RouteStep step = currentRoute[currentPatrolStep];

  switch (step.direction) {
    case DIR_FORWARD: moveForward(); break; 
    case DIR_BACKWARD: moveBackward(); break;
    case DIR_LEFT: turnLeft(); break;
    case DIR_RIGHT: turnRight(); break;
    default: stopMoving(); break;
  }

  patrolStepStartTime = millis();
}
  


// Variable to store elapsed time in current step
unsigned long stepElapsedTime = 0;

void handlePatrolStep() {
  // Night mode effects
  if (isNightTime()) {
    ledMode = 3;
  }

  if (currentPatrolStep >= currentRouteLength) {
    if (!obstacleInUse) {
      currentPatrolStep = 0; // Reset to start of route
      patrolStepStartTime = millis();
    }
  }

  if (currentPatrolStep > currentRouteLength) {
    if (obstacleInUse) {
      lcd.clear();
      lcd.print(F("Patrol done!"));
      currentMode = IDLE;    
    }
  }

  // 🔹 Check for intruder before proceeding with patrol
  if (detectIntruder()) {
    // ✅ Save time already spent on this step
    stepElapsedTime = millis() - patrolStepStartTime;

    stopMoving();  // Stop patrol movement during alert

    // Show initial alert
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Intruder!");
    delay(1000);

    // Switch LED to alert mode
    previousLedMode = ledMode;
    ledMode = 2;

    // Show ongoing warning
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Warning!");

    unsigned long intruderStart = millis();
    unsigned long intruderClearTime = millis();

    // Stay here while intruder is present
    while (true) {
      if (detectIntruder()) {
        intruderClearTime = millis();
      } else if (millis() - intruderClearTime > 500) {
        break;
      }

      updateLEDAnimations();

      if (millis() - intruderStart > 10000) {
        break;
      }
    }

    // Restore LED mode
    ledMode = previousLedMode;

    // Resume patrol from where it left off
    patrolStepStartTime = millis() - stepElapsedTime; // ⬅ continue from paused time

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Resuming...");
    delay(1000);
    lcd.clear();

    return;
  }

  // 🔹 Proceed with patrol step if no intruder
  RouteStep step = currentRoute[currentPatrolStep];
  switch (step.direction) {
    case DIR_FORWARD: moveForward(); break;
    case DIR_BACKWARD: moveBackward(); break;
    case DIR_LEFT: turnLeft(); break;
    case DIR_RIGHT: turnRight(); break;
    default: stopMoving(); break;
  }

  // Move to next step after duration
  if (millis() - patrolStepStartTime >= step.duration) {
    stopMoving();
    delay(500);
    currentPatrolStep++;
    patrolStepStartTime = millis();
  }
}

void handlePatrolRouteToggle() {
  static unsigned long lastPressTime = 0;
  static bool waitingForSecondPress = false;

  unsigned long now = millis();

  routeSelectStart = millis();
  waitingForRouteConfirm = true;

  if (!waitingForSecondPress) {
    lastPressTime = now;
    waitingForSecondPress = true;
    lcd.clear();
    lcd.print("Press again for");
    lcd.setCursor(0, 1);
    lcd.print("Obstacle Route");
    return;
  }

  if (now - lastPressTime <= 1000) {  // 1 second window
    useManualRoute = false;
    waitingForRouteConfirm = false;
    obstacleInUse = true;
    lcd.clear();
    lcd.print("Obstacle Route");
  }

  delay(1000);  // Optional display hold
  currentMode = PATROL;
  startPatrol();

  waitingForSecondPress = false;
}

unsigned long lastObstacleCheck = 0;
const unsigned long obstacleCheckInterval = 100;  // check every 100ms
bool obstacleDetected = false;
bool intruderDetected = false;


void updateObstacleDetection() {
  unsigned long now = millis();
  if (now - lastObstacleCheck >= obstacleCheckInterval) {
    lastObstacleCheck = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 30000);
    long distance = duration * 0.034 / 2;

    intruderDetected = (distance > 0 && distance < SAFE_CLEARANCE_CM);
    obstacleDetected = (distance > 0 && distance < 15);
  }
}

// void removeLastObstacleStep() {
//   if (obstacleRouteIndex > 0) {
//     obstacleRouteIndex--;
//     Serial.print("Removed last step. New index: ");
//     Serial.println(obstacleRouteIndex);
//   }
// }

void finalizeObstacleMovementLog() {
  // Only apply if we just exited from obstacle avoidance
  if (lastAutoDirection == DIR_FORWARD && autoMoveStartTime > 0) {
    unsigned long moveTime = millis() - autoMoveStartTime;

    if (moveTime >= 200) {  // Optional: ignore very short movements
      logRouteStep(DIR_FORWARD, moveTime, false);
      Serial.print("Finalized Forward Segment: ");
      Serial.print(moveTime);
      Serial.println("ms");
    }

    autoMoveStartTime = 0;
    lastAutoDirection = 255;
  }
}

void handleObstacleAvoidance() {
  if (currentMode != O_AVOIDANCE) return;
  isIdle = false;

  // Auto LED logic for night patrols
  if (isNightTime()) {
    if (!autoObstacleLED) {
      previousObstacleLED = ledMode;
      ledMode = 4;
      autoObstacleLED = true;
    }
  } else if (autoObstacleLED) {
    ledMode = previousObstacleLED;
    autoObstacleLED = false;
  }

  long distance = getUltrasonicDistance();

  if (distance < SAFE_CLEARANCE_CM) {
    // If we were moving forward, log accumulated forward time
    if (lastAutoDirection == DIR_FORWARD && autoMoveStartTime > 0) {
      unsigned long moveTime = millis() - autoMoveStartTime;
      logRouteStep(DIR_FORWARD, moveTime, false);
      autoMoveStartTime = 0;
      lastAutoDirection = 255;
    }

    stopMoving();
    delay(100);

    int bestDirection = scanForOpenPath();

    RouteStep lastStep = obstacleRoute[obstacleRouteIndex - 1];
    unsigned long lastDuration = lastStep.duration;

    if (bestDirection == -1) {
      //removeLastObstacleStep(); //moveBackward(); delay(500); stopMoving();
      rotateInPlaceRight(); delay(700); stopMoving();
      logRouteStep(DIR_RIGHT, 700, false);
      lastAutoDirection = DIR_RIGHT;
      
    } else {
      unsigned long startTime = millis();

      switch (bestDirection) {
        case 0:
          turnLeft(); delay(500); stopMoving();
          logRouteStep(DIR_LEFT, 500, false);
          lastAutoDirection = DIR_LEFT;
          break;
        case 1:
          moveForward(); delay(500); stopMoving();
          logRouteStep(DIR_FORWARD, 500, false);
          lastAutoDirection = DIR_FORWARD;
          break;
        case 2:
          turnRight(); delay(500); stopMoving();
          logRouteStep(DIR_RIGHT, 500, false);
          lastAutoDirection = DIR_RIGHT;
          break;
      }
      delay(100);
    }

  } else {
    // Forward is clear
    if (lastAutoDirection != DIR_FORWARD) {
      // New forward segment
      autoMoveStartTime = millis();
      lastAutoDirection = DIR_FORWARD;
    }

    moveForward(); delay(500); stopMoving();
    // We don't log here yet, we accumulate time until obstacle or direction change
  }

  delay(100);
}


void alertFlash() {
  for (int i = 0; i < 6; i++) {
    analogWrite(LED_PIN, 255);
    delay(200);
    analogWrite(LED_PIN, 0);
    delay(200);
  }
}

void patrolStrobe() {
  for (int i = 0; i < 10; i++) {
    analogWrite(LED_PIN, (i % 2 == 0) ? 180 : 30);
    delay(100);
  }
  analogWrite(LED_PIN, 0);  // Return to off
}

void showSensorData() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Temp: "));
  lcd.print(t);
  lcd.print(F(" C"));

  lcd.setCursor(0, 1);
  lcd.print(F("Humidity: "));
  lcd.print(h);
  lcd.print(F(" %"));
  
}


void toggleLEDMode() {
  ledMode = (ledMode + 1) % 5;  // 0 → 1 → 2 → 3 → 4 → 0

  Serial.print("LED Mode: ");
  Serial.println(ledMode);

  analogWrite(LED_PIN, 0);   // Reset LED state
  ledTimer = millis();       // Reset animation timer
  ledState = false;          // Reset blink state
  flashState = 0;            // Reset strobe state
  lastChange = millis();     // Reset strobe timer
}


void updateLEDAnimations() {
  unsigned long now = millis();

  switch (ledMode) {
    case 1:  // Welcome Fade (slow blink + soft beep)
      if (now - ledTimer >= 600) {
        ledState = !ledState;  // Toggle state
        digitalWrite(LED_PIN, ledState);  // Control LED
        digitalWrite(BUZZER_PIN, ledState);  // Match buzzer with LED
        ledTimer = now;  // Update timer
      }
      break;

    case 2:  // Alert Flash (LED + Active Buzzer)
      if (now - ledTimer >= 50) {
        ledState = !ledState;  // Toggle LED state
        digitalWrite(LED_PIN, ledState);  // Control LED
        digitalWrite(BUZZER_PIN, ledState);  // Match buzzer with LED
        ledTimer = now;  // Update timer
      }
      break;

    case 3:  // Patrol LED Strobe + Buzzer Pulse (normal rhythm)
      if (now - lastChange >= 100) {
        switch (flashState) {
          case 0:
          case 2:
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(BUZZER_PIN, HIGH);  // Buzzer ON
            break;

          case 1:
          case 3:
          case 4:
            digitalWrite(LED_PIN, LOW);
            digitalWrite(BUZZER_PIN, LOW);  // Buzzer OFF
            break;
        }

        flashState++;
        if (flashState > 4) flashState = 0;

        lastChange = now;
      }
      break;

    case 4:  // Solid ON with buzzer every 2s
      analogWrite(LED_PIN, 255);  // Full brightness

      if (now - buzzerTimer >= 2000) {
        digitalWrite(BUZZER_PIN, HIGH);  // Pulse buzzer ON
        delay(200);                       // Short beep duration
        digitalWrite(BUZZER_PIN, LOW);   // Then OFF

        buzzerTimer = now;  // Reset timer
      }
      break;
  }

  // Optional: Stop buzzer if not needed
  if (ledMode != 1 && ledMode != 2 && ledMode != 3) {
    noTone(BUZZER_PIN);
  }
}

long getUltrasonicDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30ms = 5m
  long distance = duration * 0.034 / 2;

  return constrain(distance, 0, 400);  // Cap at 400 cm
}

long lastScanDistances[3];  // Global array to store latest distances

int scanForOpenPath() {
  int angles[3] = {150, 90, 30};      // Left, Center, Right
  long distances[3];

  // Take distance readings at 3 angles
  for (int i = 0; i < 3; i++) {
    servo.write(angles[i]);
    delay(300);  // Wait for servo to settle
    distances[i] = getUltrasonicDistance();
    delay(100);
    
    // Mark unsafe directions
    if (distances[i] < SAFE_CLEARANCE_CM) {
      distances[i] = -1;  // Blocked
    }
  }

  // Return servo to center
  servo.write(90);

  // Check if all directions are blocked
  if (distances[0] == -1 && distances[1] == -1 && distances[2] == -1) {
    return -1; // All paths blocked
  }

  // Choose direction with greatest safe distance
  int bestIndex = -1;
  long maxDist = -1;
  for (int i = 0; i < 3; i++) {
    if (distances[i] > maxDist) {
      maxDist = distances[i];
      bestIndex = i;
    }
  }

  return bestIndex;  // 0 = Left, 1 = Forward, 2 = Right
}

void checkIR() {
  if (IrReceiver.decode()) {
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
      uint8_t command = IrReceiver.decodedIRData.command;
      Serial.print("IR received: ");
      Serial.println(command, HEX);
      if (command != 0xFFFFFFFF && command != 0x0) {
        Serial.print("Received command: 0x");
        Serial.println(command, HEX);

        handleIR(command);
        updateLastAction();
        nightOverride = true;
      }
    }
  }
  IrReceiver.resume();
}

void handleIR(uint8_t command) {
  // Restrict movement buttons if not in MANUAL mode
  if ((command == 0x18 || command == 0x52 ||  command == 0x08 || command == 0x5A || command == 0x1C) && currentMode != MANUAL) {
    lcd.clear();
    lcd.print(F("Manual mode only"));
    delay(800);
    lcd.clear();
    return;
  }

  switch (command) {
    // Modes
    case 0x45:  // Button "1"
      // To stop the motors after exiting patrol mode
      if (isPatrolRunning) {
        stopMoving();
        isPatrolRunning = false;
      }

      // To stop logging route
      isLoggingManualRoute = false;
      finalizeObstacleMovementLog();
      currentMode = MANUAL;
      lcd.clear();
      lcd.print(F("Mode: Manual"));
      break;

    case 0x46:  // Button "2"
      // To stop logging route
      isLoggingManualRoute = false;

      finalizeObstacleMovementLog();
      currentMode = O_AVOIDANCE;
      lcd.clear();
      lcd.print(F("Mode: O-Avoidance"));
      break;

    case 0x47:  // Button "3"
      // To stop logging route
      isLoggingManualRoute = false;

      // To stop the motors after exiting patrol mode
      if (isPatrolRunning) {
        stopMoving();
        isPatrolRunning = false;
      }

      finalizeObstacleMovementLog();
      currentMode = RETURN_HOME;
      lcd.clear();
      lcd.print(F("Mode: Return"));
      break;

    case 0x44:  // Button 4: Patrol Mode
      // To stop logging route
      isLoggingManualRoute = false;

      finalizeObstacleMovementLog();
      handlePatrolRouteToggle();  // Now handles timing and start
      break;

    case 0x40:  // Button "5"
      // To stop logging route
      isLoggingManualRoute = false;

      finalizeObstacleMovementLog();
      currentMode = IDLE;
      lcd.clear();
      lcd.print(F("Mode: Idle"));
      break;

    case 0x07:  // Button "7"
      finalizeObstacleMovementLog();
      if (currentMode == MANUAL) {
        isLoggingManualRoute = true;
        manualRouteIndex = 0;
        lcd.clear();
        lcd.print(F("Logging manual"));
        lcd.setCursor(0, 1);
        lcd.print(F("route started"));
      } else {
        lcd.clear();
        lcd.print(F("Not in manual"));
        lcd.setCursor(0, 1);
        lcd.print(F("mode!"));
      }
      break;

    // 🎮 Manual movement
    case 0x18: moveForward(); break;   // Up
    case 0x52: moveBackward(); break;  // Down
    case 0x08: turnLeft(); break;      // Left
    case 0x5A: turnRight(); break;     // Right
    case 0x1C: // OK button
      if (currentMode == MANUAL) {
        stopMoving();  // Stops motors
        lcd.setCursor(0, 1);
        lcd.print(F("                "));

        // Only log the movement if it's meaningful
        if (lastDirection != DIR_NONE) {
          logRouteStep(lastDirection, duration, true);
        }

        // OPTION 1 — Keep logging active
        if (isLoggingManualRoute) {
          lcd.clear();
          lcd.print("Step logged");
          delay(1000);
          lcd.clear();
        }

    
  }
  break;

    // ⚙️ Features
    case 0x43: toggleLEDMode(); break;  // Button "6"
    case 0x0D: toggleSpeed(); break;    // Button "#"

    // 🧪 Sensor Display
    case 0x15: showSensorData(); break;  // Button "8"
    case 0x09: showLightLevel(); break;  // Button "9"
    case 0x16: resetLogs(); break;       // Button "*"

    default:
      lcd.clear();
      lcd.print(F("Unknown command"));
      delay(200);
      lcd.setCursor(0, 0);
      lcd.print(F("                "));
      break;
  }
}
void checkIdleTimeout() {
  // Check if inactive for too long, enter sleep
}

void handleManual() {
  isIdle = false;
  // Manual control logic with IR input
}



void updatePatrolProgress(uint8_t currentStep, uint8_t totalSteps) {
  int blocks = map(currentStep, 0, totalSteps, 0, 16);
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++) {
    lcd.write(i < blocks ? 255 : ' ');
  }
}


void handleIdle() {
  isIdle = true;
  stopMoving();  // Ensure motors are off
  ledMode = 0;   // Turn off LED effects
  noTone(BUZZER_PIN); // Silence buzzer if it was active
}

void enterSleepMode() {
  // Power-saving mode
}

void updateLastAction() {
  lastActionTime = millis();
}

void showLightLevel() {
  int lightValue = analogRead(LDR_PIN);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Light Level:");
  lcd.setCursor(0, 1);
  lcd.print(lightValue);

  delay(2000); 
  lcd.setCursor(0, 1);
  lcd.print(F("                "));
  
}

void resetLogs() {
  // Clear manual route data
  for (uint8_t i = 0; i < manualRouteIndex; i++) {
    manualRoute[i] = { DIR_NONE, 0 };
  }
  manualRouteIndex = 0;
  isLoggingManualRoute = false;

  // Clear obstacle route data
  for (uint8_t i = 0; i < obstacleRouteIndex; i++) {
    obstacleRoute[i] = { DIR_NONE, 0 };
  }
  obstacleRouteIndex = 0;
  autoMoveStartTime = 0;
  lastAutoDirection = 255;
  autoRouteCount = 0;

  // Display feedback
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Logs Reset");
  delay(1000);
}


void toggleSpeed() {
  if (currentSpeed == 100) {
    currentSpeed = 150; // Normal
    lcd.setCursor(0, 1);
    lcd.print(F("Speed: Normal   "));
  } else if (currentSpeed == 150) {
    currentSpeed = 200; // Fast
    lcd.setCursor(0, 1);
    lcd.print(F("Speed: Fast     "));
  } else {
    currentSpeed = 100; // Slow
    lcd.setCursor(0, 1);
    lcd.print(F("Speed: Slow     "));
  }
  Serial.print(currentSpeed);
}

// ========== Setup ==========
void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();
  dht.begin();
  delay(500);
  servo.attach(SERVO_PIN); 

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2 , OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("  ModuBot v1.0"));
  lcd.setCursor(0, 1);
  lcd.print(F(" Initializing..."));
  delay(1000);

  // Simple progress bar
  lcd.setCursor(0, 1);
  for (int i = 0; i < 16; i++) {
    lcd.setCursor(i, 1);
    lcd.write(byte(255));  // Full block character
    delay(100);
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("   ModuBot is"));
  lcd.setCursor(0, 1);
  lcd.print(F("     Ready!"));
  delay(1500);
  lcd.clear();

  IrReceiver.begin(IR_PIN, false);

  welcomeAnimation(); 

  lcd.print(F("Mode: IDLE"));

}

// ========== Loop ========== //
void loop() { 
  if (currentMode == MANUAL && !isLoggingManualRoute) {
      updateObstacleDetection();
  }
  if (IrReceiver.isIdle()) irReady = true;
  if (!irReady) return; // wait for IR to settle

  checkIR();
  checkIdleTimeout();

  if (currentMode == MANUAL) {
    if (!isLoggingManualRoute && obstacleDetected ) {
      stopMoving();
      delay(30);
      lcd.setCursor(0, 1);
      lcd.print(F("Obstacle ahead!"));
      moveBackward();
      delay(200);
      stopMoving();
    }

  }

  if (waitingForRouteConfirm && millis() - routeSelectStart > 1000) {
    useManualRoute = true;
    lcd.clear();
    lcd.print("Manual Route");
    delay(1000);  // Optional delay
    currentMode = PATROL;
    startPatrol();

    waitingForRouteConfirm = false;
  }

  if (currentMode == PATROL && isPatrolRunning) {
  handlePatrolStep();
  }
  // Safety print


  switch (currentMode) {
    case MANUAL:
      handleManual();
      break;

    case O_AVOIDANCE:
      handleObstacleAvoidance();
      break;

    case RETURN_HOME:
      returnHome();
      break;

    case PATROL:
      handlePatrolStep();
      break;

    case IDLE:
      handleIdle();
      break;

    case SLEEP:
      enterSleepMode();
      break;
  }

  updateLEDAnimations();
}
