#include <BluetoothSerial.h>

// Définition des broches pour les moteurs
#define MOTOR_FL_PWM 25
#define MOTOR_FL_IN1 26
#define MOTOR_FL_IN2 27
#define MOTOR_BL_PWM 14
#define MOTOR_BL_IN1 12
#define MOTOR_BL_IN2 13
#define MOTOR_FR_PWM 15
#define MOTOR_FR_IN1 2
#define MOTOR_FR_IN2 4
#define MOTOR_BR_PWM 5
#define MOTOR_BR_IN1 18
#define MOTOR_BR_IN2 19

#define MAX_SPEED 255
#define MIN_SPEED -255

BluetoothSerial SerialBT;

void processCommand(const String& command);
void setMotorSpeed(int pwmPin, int in1Pin, int in2Pin, int speed);
int normalizeSpeed(int speed);

void setup() {
  Serial.begin(115200);
  SerialBT.begin("OMNI_PLATFORM");

  // Configuration des broches des moteurs
  pinMode(MOTOR_FL_PWM, OUTPUT);
  pinMode(MOTOR_FL_IN1, OUTPUT);
  pinMode(MOTOR_FL_IN2, OUTPUT);
  pinMode(MOTOR_BL_PWM, OUTPUT);
  pinMode(MOTOR_BL_IN1, OUTPUT);
  pinMode(MOTOR_BL_IN2, OUTPUT);
  pinMode(MOTOR_FR_PWM, OUTPUT);
  pinMode(MOTOR_FR_IN1, OUTPUT);
  pinMode(MOTOR_FR_IN2, OUTPUT);
  pinMode(MOTOR_BR_PWM, OUTPUT);
  pinMode(MOTOR_BR_IN1, OUTPUT);
  pinMode(MOTOR_BR_IN2, OUTPUT);

  Serial.println("ESP32 Omnidirectional Platform ready. Pair with Bluetooth to control.");
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    Serial.println("Received: " + command);
    processCommand(command);
  }
}

void processCommand(const String& command) {
  int x1, y1, x2, y2;
  if (sscanf(command.c_str(), "%d %d %d %d", &x1, &y1, &x2, &y2) == 4) {
    // Calcul des vitesses des moteurs
    int speedFL = normalizeSpeed(x1 + y1 + x2);
    int speedFR = normalizeSpeed(-x1 + y1 - x2);
    int speedBL = normalizeSpeed(-x1 + y1 + x2);
    int speedBR = normalizeSpeed(x1 + y1 - x2);

    // Application des vitesses aux moteurs
    setMotorSpeed(MOTOR_FL_PWM, MOTOR_FL_IN1, MOTOR_FL_IN2, speedFL);
    setMotorSpeed(MOTOR_FR_PWM, MOTOR_FR_IN1, MOTOR_FR_IN2, speedFR);
    setMotorSpeed(MOTOR_BL_PWM, MOTOR_BL_IN1, MOTOR_BL_IN2, speedBL);
    setMotorSpeed(MOTOR_BR_PWM, MOTOR_BR_IN1, MOTOR_BR_IN2, speedBR);

    Serial.printf("Motors: FL=%d, FR=%d, BL=%d, BR=%d\n", speedFL, speedFR, speedBL, speedBR);
  } else {
    Serial.println("Invalid command format");
  }
}

void setMotorSpeed(int pwmPin, int in1Pin, int in2Pin, int speed) {
  digitalWrite(in1Pin, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2Pin, speed >= 0 ? LOW : HIGH);
  analogWrite(pwmPin, abs(speed));
}

int normalizeSpeed(int speed) {
  return constrain(speed, MIN_SPEED, MAX_SPEED);
}