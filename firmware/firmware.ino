#include <Servo.h>

#define GRIPPER_PIN 11   
#define TWIST_PIN 10  
#define WRIST_PIN 9
#define ELBOW_PIN 6   
#define SHOULDER_PIN 5
#define BASE_PIN 3

Servo gripper, twist, wrist, elbow, shoulder, base;

const int MIN_ANGLE = 0;
const int MAX_ANGLE = 180;
const int GRIPPER_MIN = 0; // open
const int GRIPPER_MAX = 80; // close

int currentAngles[6] = {0, 0, 0, 0, 0, 80};  // b, s, e, w, t, g

String inputBuffer = "";
bool newCommand = false;

void setup() {
  gripper.attach(GRIPPER_PIN);
  twist.attach(TWIST_PIN);
  wrist.attach(WRIST_PIN);
  elbow.attach(ELBOW_PIN);
  shoulder.attach(SHOULDER_PIN);
  base.attach(BASE_PIN);

  Serial.begin(115200);
  
  base.write(currentAngles[0]);
  shoulder.write(currentAngles[1]);
  elbow.write(currentAngles[2]);
  wrist.write(currentAngles[3]);
  twist.write(currentAngles[4]);
  gripper.write(currentAngles[5]);
  
  delay(500);
  sendFeedback();
}

void loop() {
  readSerialCommand();
  
  if (newCommand) {
    parseCommand(inputBuffer);
    inputBuffer = "";
    newCommand = false;
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