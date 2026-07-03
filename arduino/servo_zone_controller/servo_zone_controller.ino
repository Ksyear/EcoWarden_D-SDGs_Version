#include <Servo.h>

Servo cameraServo;

const int kServoPin = 9;
const int kMinAngle = 0;
const int kMaxAngle = 180;

String line;

void setup() {
  Serial.begin(115200);
  cameraServo.attach(kServoPin);
  cameraServo.write(90);
  Serial.println("OK BOOT");
}

void loop() {
  while (Serial.available() > 0) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      handleLine(line);
      line = "";
    } else if (line.length() < 32) {
      line += ch;
    }
  }
}

void handleLine(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  int angle = 90;
  if (cmd.charAt(0) == 'A' || cmd.charAt(0) == 'a') {
    angle = cmd.substring(1).toInt();
  } else {
    angle = cmd.toInt();
  }

  if (angle < kMinAngle || angle > kMaxAngle) {
    Serial.println("ERR RANGE");
    return;
  }

  cameraServo.write(angle);
  Serial.print("OK ");
  Serial.println(angle);
}
