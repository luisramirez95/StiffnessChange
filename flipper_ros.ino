// ---------------- PIN DEFINITIONS ----------------
#define MOTOR_B_IN1 3
#define MOTOR_B_IN2 2
#define MOTOR_B_PWM 5
#define MOTOR_STBY 4

#define VALVE_A_PIN 7
#define VALVE_B_PIN 6

#define PRESSURE_PIN A0

// -------------- SENSOR CALIBRATION --------------
float zeroVoltage = 0.5;
const float V_SUPPLY = 5.0;
const float V_SPAN = 4.0;
const float PSI_SPAN = 15.0;

// -------------- TARGET STATES -------------------
float targetPSI = 0;     // active pressure target (2 or 10)
bool holding = false;    // true = maintain PSI
bool quickInflating = false; // one-time fast return from -10 PSI to -2 PSI

// -------------- FUNCTION DECLARATIONS -----------
float readPressurePSI();
float calibrateZeroVoltage();
void MotorControl(int PWMVal);
void ValveA(bool open);
void ValveB(bool open);

void initPressure(float psi);
void releasePressure();
void updateHoldLogic();

// ---------------------- SETUP --------------------
void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);

  pinMode(VALVE_A_PIN, OUTPUT);
  pinMode(VALVE_B_PIN, OUTPUT);

  ValveA(false);
  ValveB(false);

  zeroVoltage = calibrateZeroVoltage();
  Serial.print("Zero Voltage: ");
  Serial.print(zeroVoltage, 3);
  Serial.println(" V");
  float psi = readPressurePSI();
  Serial.print("Pressure: ");
  Serial.print(psi, 2);
  Serial.println(" PSI");
  Serial.println("Ready. Send '2', '10', or 'release'");
}

// ----------------------- LOOP --------------------
void loop() {
  
  // Read serial command from ROS2
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "2") {
      initPressure(-2.0);
    }
    else if (cmd == "10") {
      initPressure(-10.0);
    }
    else if (cmd == "release") {
      releasePressure();
    }
  }

  // If holding, run the holding controller
  if (holding) updateHoldLogic();
}


// -------------------------------------------------
// HELPER: Begin pressure regulation mode
// -------------------------------------------------
void initPressure(float psi) {
  bool switchingFromMinus10ToMinus2 =
      holding && targetPSI == -10.0 && psi == -2.0;

  targetPSI = psi;
  holding = true;
  quickInflating = switchingFromMinus10ToMinus2;

  Serial.print("Entering hold mode at ");
  Serial.print(targetPSI);
  Serial.println(" PSI");

  ValveA(false);   // close
  ValveB(true);   // open
  MotorControl(0); // motor off initially
}


// -------------------------------------------------
// HELPER: Release to atmosphere
// -------------------------------------------------
void releasePressure() {
  Serial.println("Releasing pressure…");

  holding = false;
  quickInflating = false;
  ValveA(true);    // open
  ValveB(true);    // open
  MotorControl(0);
}


// -------------------------------------------------
// HOLD MODE LOGIC: simple open-loop to maintain 2 or 10 PSI
// -------------------------------------------------
void updateHoldLogic() {
  float psi = readPressurePSI();
  const char* controllerState;
  int commandedPumpPWM;
  const char* valveAState;
  const char* valveBState;

  // One-time active inflation when changing from -10 PSI to -2 PSI.
  if (quickInflating && psi < targetPSI - 0.3) {
    ValveA(true);
    ValveB(true);
    MotorControl(200);
    controllerState = "RELEASING";
    commandedPumpPWM = 200;
    valveAState = "OPEN";
    valveBState = "CLOSED";
  }

  // Not enough vacuum.
  // Example: target = -10, current = -5
  // Need to suck more.
  else if (psi > targetPSI + 0.3) {
    quickInflating = false;
    ValveA(false);
    ValveB(false);
    MotorControl(200);
    controllerState = "SUCKING";
    commandedPumpPWM = 200;
    valveAState = "CLOSED";
    valveBState = "CLOSED";
  }

  // Too much vacuum.
  // Example: target = -2, current = -5
  // Need to release vacuum.
  else if (psi < targetPSI - 0.3) {
    quickInflating = false;
    ValveA(true);
    ValveB(true);
    MotorControl(0);
    controllerState = "RELEASING";
    commandedPumpPWM = 0;
    valveAState = "OPEN";
    valveBState = "OPEN";
  }

  // Within ±0.3 PSI of target -> hold
  else {
    quickInflating = false;
    ValveA(false);
    ValveB(true);
    MotorControl(0);
    controllerState = "HOLDING";
    commandedPumpPWM = 0;
    valveAState = "CLOSED";
    valveBState = "OPEN";
  }

  Serial.print("PSI:");
  Serial.print(psi, 2);
  Serial.print(" Target:");
  Serial.print(targetPSI, 2);
  Serial.print(" State:");
  Serial.print(controllerState);
  Serial.print(" Pump:");
  Serial.print(commandedPumpPWM);
  Serial.print(" ValveA:");
  Serial.print(valveAState);
  Serial.print(" ValveB:");
  Serial.println(valveBState);

  delay(120);
}


// -------------------------------------------------
// LOW-LEVEL HARDWARE FUNCTIONS
// -------------------------------------------------
void MotorControl(int PWMVal) {
  if (PWMVal > 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_PWM, PWMVal);
  } else {
    analogWrite(MOTOR_B_PWM, 0);
  }
}

void ValveA(bool open) {
  digitalWrite(VALVE_A_PIN, open ? HIGH : LOW);
}

void ValveB(bool open) {
  digitalWrite(VALVE_B_PIN, open ? HIGH : LOW);
}


// ---------------- PRESSURE SENSOR ----------------
float readPressurePSI() {
  int raw = analogRead(PRESSURE_PIN);
  float voltage = raw * (V_SUPPLY / 1023.0);
  // This sensor port orientation rises in voltage as physical pressure becomes
  // more negative, so invert the calibrated value to report gauge pressure.
  return -((voltage - zeroVoltage) / V_SPAN) * PSI_SPAN;
}

float calibrateZeroVoltage() {
  float sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(PRESSURE_PIN) * (V_SUPPLY / 1023.0);
    delay(10);
  }
  return sum / 20.0;
}
