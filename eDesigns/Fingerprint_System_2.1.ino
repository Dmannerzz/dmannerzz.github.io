// This version has been improved upon with Software Serial

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Fingerprint.h>
// #include <SoftwareSerial.h>

// -------------------- OLED Setup --------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// // -------------------- Pins --------------------
#define GREEN_LED 6
#define RED_LED 7
#define BUZZER 8
#define ENTER_BUTTON 4   // Enter button
#define UP_BUTTON 9   // Up button
#define DOWN_BUTTON 5     // Down button

// -------------------- Fingerprint Setup --------------------
// Fingerprint Setup using hardware serial
Adafruit_Fingerprint finger(&Serial);
// SoftwareSerial fingerSerial(2, 3); // RX, TX
// Adafruit_Fingerprint finger(&fingerSerial);
bool sensorVerified = false;

// -------------------- Menu Setup --------------------
int menuIndex = 0;
const int menuSize = 4;  
const int visibleItems = 3;  // how many items fit on screen


const char* menuItems[] = {
  "Enroll",
  "Test",
  "Delete",
  "Log"    
};

// -------------------- Fingerprint ID Tracker --------------------
#define MAX_USERS 10   // adjust to your sensor capacity
int enrolledIDs[MAX_USERS];
int enrolledCount = 0;

// Helper function to check if an ID is already enrolled
bool isEnrolled(int id) {
  for (int i = 0; i < enrolledCount; i++) {
    if (enrolledIDs[i] == id) return true;
  }
  return false;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(57600);   // hardware UART for fingerprint sensor
  Serial.setTimeout(1000);
  
  // Pins
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, HIGH);
  pinMode(UP_BUTTON, INPUT_PULLUP);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  pinMode(ENTER_BUTTON, INPUT_PULLUP);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  
  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    //Serial.println(F("OLED init failed"));
    for(;;);
  }
  
  delay(100);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 20);
  display.println(F("Hello,"));
  display.println(F("Welcome"));
  display.display();

  // Beep buzzer with a simple tune at startup
  playStartupTune();

  //delay(500);

  // Initialize fingerprint sensor
  finger.begin(57600);
  delay(500);
  
  for (int i = 0; i < 5; i++) {
    if (finger.verifyPassword()) {
      digitalWrite(GREEN_LED, HIGH);
      delay(500);
      digitalWrite(GREEN_LED, LOW);
      sensorVerified = true;
      break;
    } else {
      digitalWrite(RED_LED, HIGH);
      delay(2000);
      digitalWrite(RED_LED, LOW);
      delay(500);
    }
  }
  if (!sensorVerified) {
    digitalWrite(RED_LED, HIGH);
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println(F("Sensor not"));
    display.println(F("detected!"));
    display.display();
    while (1) { delay(1); }
  }
  
}

// -------------------- Startup Tune --------------------
void playStartupTune() {
  // Long "taaaaaan"
  digitalWrite(BUZZER, LOW);
  delay(600);
  digitalWrite(BUZZER, HIGH);
  delay(100);

  // Short "taan"
  digitalWrite(BUZZER, LOW);
  delay(200);
  digitalWrite(BUZZER, HIGH);
  delay(100);

  // Short "taan"
  digitalWrite(BUZZER, LOW);
  delay(200);
  digitalWrite(BUZZER, HIGH);
  delay(100);
}

// -------------------- Loop --------------------
void loop() {

  showMenu();

  // DOWN button
  if (digitalRead(DOWN_BUTTON) == LOW) {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);

    menuIndex++;
    if (menuIndex >= menuSize) menuIndex = 0;

    delay(200); // debounce
  }

  // UP button
  if (digitalRead(UP_BUTTON) == LOW) {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);

    menuIndex--;
    if (menuIndex < 0) menuIndex = menuSize - 1;

    delay(200); // debounce
  }

  // ENTER button
  if (digitalRead(ENTER_BUTTON) == LOW) {
    digitalWrite(BUZZER, LOW);
    delay(100);
    digitalWrite(BUZZER, HIGH);

    switch (menuIndex) {
      case 0:
        enrollFingerprint();
        break;

      case 1:
        confirmFingerprint();
        break;

      case 2:
        deleteID();
        break;

      case 3:
        showLog();   // new Log option
        break;
    }

    delay(300);
  }
}

void showMenu() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Menu"));

  // calculate the first item to show
  int start = 0;
  if (menuIndex >= visibleItems) {
    start = menuIndex - visibleItems + 1;
  }

  for (int i = 0; i < visibleItems && (start + i) < menuSize; i++) {
    int itemIndex = start + i;

    if (itemIndex == menuIndex) {
      display.setTextColor(BLACK, WHITE);  // highlight
    } else {
      display.setTextColor(WHITE);
    }

    display.setCursor(20, display.getCursorY());
    display.println(menuItems[itemIndex]);
  }

  display.setTextColor(WHITE);
  display.display();
}

void deleteID() {
  if (enrolledCount == 0) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.println(F("No"));
    display.setCursor(0, 25);
    display.println(F("Prints"));
    display.setCursor(0, 40);
    display.println(F("Enrolled"));
    display.display();
    delay(2000);
    return;
  }

  int index = 0;
  int id = enrolledIDs[index];

  while (true) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(F("Selected ID: "));
    display.println(id);
    display.setCursor(0, 40);
    display.println(F("UP/DOWN to change"));
    display.setCursor(0, 55);
    display.println(F("ENTER to delete"));
    display.display();

    // UP button → next enrolled ID
    if (digitalRead(UP_BUTTON) == LOW) {
      index++;
      if (index >= enrolledCount) index = 0;
      id = enrolledIDs[index];
      delay(200);
    }

    // DOWN button → previous enrolled ID
    if (digitalRead(DOWN_BUTTON) == LOW) {
      index--;
      if (index < 0) index = enrolledCount - 1;
      id = enrolledIDs[index];
      delay(200);
    }

    // ENTER button → delete chosen ID
    if (digitalRead(ENTER_BUTTON) == LOW) {
      if (finger.deleteModel(id) == FINGERPRINT_OK) {
        display.clearDisplay();
        display.setCursor(0, 20);
        display.print(F("Deleted ID: "));
        display.println(id);
        display.display();

        // Remove from local list
        for (int i = index; i < enrolledCount - 1; i++) {
          enrolledIDs[i] = enrolledIDs[i + 1];
        }
        enrolledCount--;

        // Reset index safely
        if (enrolledCount > 0) {
          if (index >= enrolledCount) index = 0;
          id = enrolledIDs[index];
        }
      } else {
        display.clearDisplay();
        display.setCursor(0, 20);
        display.println(F("Delete Failed"));
        display.display();
      }

      delay(2000);
      break; // exit after deletion
    }
  }
}

void showLog() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("Log"));

  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print(F("Enrolled IDs: "));
  display.println(enrolledCount);

  // List the actual IDs
  display.setCursor(0, 35);
  display.print(F("IDs: "));
  for (int i = 0; i < enrolledCount; i++) {
    display.print(enrolledIDs[i]);
    if (i < enrolledCount - 1) {
      display.print(F(", ")); // comma between IDs
    }
  }

  display.display();
  delay(3000); // show for 3 seconds before returning
}

uint8_t getFingerprintImage() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      return FINGERPRINT_OK;
    case FINGERPRINT_NOFINGER:
      return FINGERPRINT_NOFINGER;
    case FINGERPRINT_PACKETRECIEVEERR:
      return FINGERPRINT_PACKETRECIEVEERR;
    case FINGERPRINT_IMAGEFAIL:
      return FINGERPRINT_IMAGEFAIL;
    default:
      return p;
  }
}

void enrollFingerprint() {
  int id = 1;  // start at 1

  // Let user pick an unused ID
  while (true) {
    // Ensure current ID is free
    while (isEnrolled(id)) {
      id++;
      if (id > MAX_USERS) id = 1; // wrap around
    }

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(F("ID: "));
    display.println(id);
    display.setTextSize(1);
    display.setCursor(0, 40);
    display.println(F("UP/DOWN to change"));
    display.setCursor(0, 55);
    display.println(F("ENTER to confirm"));
    display.display();

    // UP button → next free ID
    if (digitalRead(UP_BUTTON) == LOW) {
      do {
        id++;
        if (id > MAX_USERS) id = 1;
      } while (isEnrolled(id));  // skip enrolled IDs
      delay(200);
    }

    // DOWN button → previous free ID
    if (digitalRead(DOWN_BUTTON) == LOW) {
      do {
        id--;
        if (id < 1) id = MAX_USERS;
      } while (isEnrolled(id));  // skip enrolled IDs
      delay(200);
    }

    // ENTER button → confirm
    if (digitalRead(ENTER_BUTTON) == LOW) {
      delay(300);
      break;
    }
  }

  // Prompt: Place finger
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(F("Place"));
  display.setCursor(0, 40);
  display.println(F("finger..."));
  display.display();

  while (getFingerprintImage() != FINGERPRINT_OK) {
    delay(100);
  }

  if (finger.image2Tz(1) != FINGERPRINT_OK) {
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println(F("Error converting image"));
    display.display();
    delay(2000);
    return;
  }

  // Prompt: Remove finger
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(F("Remove"));
  display.setCursor(0, 40);
  display.println(F("finger"));
  display.display();
  delay(2000);

  // Prompt: Place same finger again
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(F("Place same"));
  display.setCursor(0, 40);
  display.println(F("finger"));
  display.display();

  while (getFingerprintImage() != FINGERPRINT_OK) {
    delay(100);
  }

  if (finger.image2Tz(2) != FINGERPRINT_OK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("Error converting image"));
    display.display();
    delay(2000);
    return;
  }

  if (finger.createModel() != FINGERPRINT_OK) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(F("Capture"));
    display.display();
    display.setCursor(0, 40);
    display.println(F("failed"));
    display.display();
    digitalWrite(RED_LED, HIGH);
    delay(2000);
    digitalWrite(RED_LED, LOW);
    return;
  }

  if (finger.storeModel(id) == FINGERPRINT_OK) {
    // Track the enrolled ID locally (with bounds check)
    if (enrolledCount < MAX_USERS) {
      enrolledIDs[enrolledCount++] = id;
    }

    display.clearDisplay();
    display.setCursor(0, 20);
    display.print(F("Stored at ID: "));
    display.println(id);
    display.display();

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BUZZER, LOW);
    delay(1500);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER, HIGH);
  } else {
    display.clearDisplay();
    display.setCursor(0, 20);
    display.println(F("Error storing model"));
    display.display();
    delay(2000);
  }
}

void confirmFingerprint() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.println(F("Place"));
  display.display();
  display.setCursor(0, 30);
  display.println(F("finger"));
  display.display();
  display.setCursor(0, 50);
  display.println(F("to test"));
  display.display();

  uint8_t p = getFingerprintImage();

  // Wait until finger is placed
  while (p != FINGERPRINT_OK) {
    p = getFingerprintImage();
    delay(100);
  }

  // Convert the image to template
  if (finger.image2Tz(1) != FINGERPRINT_OK) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(F("Error converting"));
    display.display();
    digitalWrite(RED_LED, HIGH);
    delay(500);
    digitalWrite(RED_LED, LOW);
    return;
  }

  // Search for a matching fingerprint
  int result = finger.fingerFastSearch();

  if (result == FINGERPRINT_OK) {
    // Normalize confidence to percentage
    int confidencePercent = map(finger.confidence, 0, 255, 0, 100);

    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(BUZZER, LOW);

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.println(F("Match"));
    display.display();
    display.setTextSize(2);
    display.setCursor(0, 30);
    display.println(F("Found!"));
    display.display();
    delay(2000);

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.print(F("ID: "));
    display.println(finger.fingerID);   // <-- shows the ID
    display.display();
    delay(2000);

    // display.clearDisplay();
    // display.setTextSize(2);
    // display.setCursor(0, 20);
    // display.print(F("Confidence: "));
    // display.print(confidencePercent);   // <-- show mapped percentage
    // display.println(F("%"));
    // display.display();
    // delay(1000);

    digitalWrite(GREEN_LED, LOW);
    digitalWrite(BUZZER, HIGH);
  } else {
    // No match found
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(F("Invalid!"));
    display.display();

    digitalWrite(RED_LED, HIGH);
    delay(1000);
    digitalWrite(RED_LED, LOW);
  }
}