#include <Servo.h>
#include <EEPROM.h>

#define GRIPPER_PIN 11   
#define TWIST_PIN 10  
#define WRIST_PIN 9
#define ELBOW_PIN 6   
#define SHOULDER_PIN 5
#define BASE_PIN 3

Servo gripper, twist, wrist, elbow, shoulder, base;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int GRIPPER_MIN = 30; // open
const int GRIPPER_MAX = 80; // close

// EEPROM layout: 6 angle bytes at addresses 0-5, magic byte at address 6
#define EEPROM_BASE_ADDR  0
#define EEPROM_MAGIC_ADDR 6
#define EEPROM_MAGIC_VAL  0xAB

// Safe neutral start position (where arm typically rests when unpowered)
int currentAngles[6] = {0, 0, 0, 0, 0, 80};   // b, s, e, w, t, g

// Target home position – arm moves here gradually on startup
const int homeAngles[6] = {90, 130, 180, 180, 180, 80};  // b, s, e, w, t, g

String inputBuffer = "";
bool newCommand = false;
unsigned long lastSaveTime = 0;
bool anglesDirty = false;

void setup() {
  gripper.attach(GRIPPER_PIN);
  twist.attach(TWIST_PIN);
  wrist.attach(WRIST_PIN);
  elbow.attach(ELBOW_PIN);
  shoulder.attach(SHOULDER_PIN);
  base.attach(BASE_PIN);

  Serial.begin(115200);
  
  // Restore last known angles from EEPROM (falls back to defaults on first boot)
  loadAnglesFromEEPROM();

  // Hold the restored position so the arm doesn't jerk
  for (int i = 0; i < 6; i++) {
    writeToServo(i, currentAngles[i]);
  }
  delay(300);

  // Gradually sweep to home position over ~10 seconds (100 steps × 100 ms)
  smoothMoveTo(homeAngles, 100, 100);

  // Persist the home position immediately
  saveAnglesToEEPROM();

  delay(200);
  sendFeedback();
}

void loop() {
  readSerialCommand();
  
  if (newCommand) {
    parseCommand(inputBuffer);
    inputBuffer = "";
    newCommand = false;
  }

  // Save to EEPROM 1 second after the last movement (debounced)
  if (anglesDirty && (millis() - lastSaveTime >= 1000)) {
    saveAnglesToEEPROM();
    anglesDirty = false;
  }
}

void loadAnglesFromEEPROM() {
  if (EEPROM.read(EEPROM_MAGIC_ADDR) != EEPROM_MAGIC_VAL) return; // first boot, no valid data
  for (int i = 0; i < 5; i++) {  // joints only (b,s,e,w,t) — gripper always starts at home
    currentAngles[i] = EEPROM.read(EEPROM_BASE_ADDR + i);
  }
  // currentAngles[5] (gripper) intentionally keeps its init value (80 = closed)
  // so the gripper never snaps open unexpectedly on power-up or reconnect
}

void saveAnglesToEEPROM() {
  for (int i = 0; i < 6; i++) {
    EEPROM.update(EEPROM_BASE_ADDR + i, (uint8_t)currentAngles[i]); // only writes if value changed
  }
  EEPROM.update(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VAL);
  lastSaveTime = millis();
}

// Gradually move all servos from currentAngles to targetAngles
void smoothMoveTo(const int targetAngles[], int steps, int stepDelay_ms) {
  int startAngles[6];
  for (int i = 0; i < 6; i++) startAngles[i] = currentAngles[i];

  for (int step = 1; step <= steps; step++) {
    for (int i = 0; i < 6; i++) {
      int angle = startAngles[i] + (int)((float)(targetAngles[i] - startAngles[i]) * step / steps);
      currentAngles[i] = angle;
      writeToServo(i, angle);
    }
    delay(stepDelay_ms);
  }

  // Guarantee exact target values
  for (int i = 0; i < 6; i++) {
    currentAngles[i] = targetAngles[i];
    writeToServo(i, targetAngles[i]);
  }
}

void readSerialCommand() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      if (inputBuffer.length() > 0) {
        newCommand = true;
      }
    } else {
      inputBuffer += inChar;
    }
  }
}

void parseCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  
  if (cmd == "get") {
    sendFeedback();
    return;
  }
  
  int startPos = 0;
  while (startPos < cmd.length()) {
    // Find servo identifier
    char servoChar = cmd.charAt(startPos);
    int servoIndex = getServoIndex(servoChar);
    
    if (servoIndex == -1) {
      startPos++;
      continue;
    }
    
    int commaPos = cmd.indexOf(',', startPos);
    if (commaPos == -1) {
      commaPos = cmd.length();
    }
    
    String angleStr = cmd.substring(startPos + 1, commaPos);
    int angle = angleStr.toInt();
    
    if (servoIndex == 5) { // Gripper
      angle = constrain(angle, GRIPPER_MIN, GRIPPER_MAX);
    } else {
      angle = constrain(angle, MIN_ANGLE, MAX_ANGLE);
    }
    
    currentAngles[servoIndex] = angle;
    writeToServo(servoIndex, angle);
    anglesDirty = true;
    lastSaveTime = millis(); // reset 1-second cooldown on each new command
    
    startPos = commaPos + 1;
  }
  
  Serial.println("ACK");
}

int getServoIndex(char c) {
  switch(c) {
    case 'b': return 0; 
    case 's': return 1;
    case 'e': return 2;  
    case 'w': return 3; 
    case 't': return 4; 
    case 'g': return 5; 
    default: return -1;
  }
}

void writeToServo(int index, int angle) {
  switch(index) {
    case 0: base.write(angle); break;
    case 1: shoulder.write(angle); break;
    case 2: elbow.write(angle); break;
    case 3: wrist.write(angle); break;
    case 4: twist.write(angle); break;
    case 5: gripper.write(angle); break;
  }
}

void sendFeedback() {
  Serial.print("FEEDBACK:");
  Serial.print("b"); Serial.print(currentAngles[0]);
  Serial.print(",s"); Serial.print(currentAngles[1]);
  Serial.print(",e"); Serial.print(currentAngles[2]);
  Serial.print(",w"); Serial.print(currentAngles[3]);
  Serial.print(",t"); Serial.print(currentAngles[4]);
  Serial.print(",g"); Serial.print(currentAngles[5]);
  Serial.println();
}