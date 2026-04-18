// Simple Motor Test Sketch - Dual Motor Driver (Front & Back)
// Tests all 4 motors independently

// FRONT MOTORS (Motor Driver 1)
const int FRONT_ENA = 3;    // Front left PWM
const int FRONT_IN1 = 2;    // Front left forward
const int FRONT_IN2 = 4;    // Front left backward
const int FRONT_ENB = 5;    // Front right PWM
const int FRONT_IN3 = 7;    // Front right forward
const int FRONT_IN4 = 8;    // Front right backward

// BACK MOTORS (Motor Driver 2)
const int BACK_ENA = 6;     // Back left PWM
const int BACK_IN1 = 10;    // Back left forward
const int BACK_IN2 = 11;    // Back left backward
const int BACK_ENB = 9;     // Back right PWM
const int BACK_IN3 = 12;    // Back right forward
const int BACK_IN4 = 13;    // Back right backward

void setup() {
  Serial.begin(115200);
  delay(500);
  
  // Set all FRONT pins as outputs
  pinMode(FRONT_ENA, OUTPUT);
  pinMode(FRONT_ENB, OUTPUT);
  pinMode(FRONT_IN1, OUTPUT);
  pinMode(FRONT_IN2, OUTPUT);
  pinMode(FRONT_IN3, OUTPUT);
  pinMode(FRONT_IN4, OUTPUT);
  
  // Set all BACK pins as outputs
  pinMode(BACK_ENA, OUTPUT);
  pinMode(BACK_ENB, OUTPUT);
  pinMode(BACK_IN1, OUTPUT);
  pinMode(BACK_IN2, OUTPUT);
  pinMode(BACK_IN3, OUTPUT);
  pinMode(BACK_IN4, OUTPUT);
  
  // Start with all motors stopped
  stopAllMotors();
  
  Serial.println("\n=== 4-MOTOR TEST ===");
  Serial.println("Testing FRONT LEFT motor first...");
  delay(1000);
}

void loop() {
  // ===== FRONT LEFT MOTOR =====
  Serial.println("\n[FRONT-LEFT] FORWARD (PWM=220)");
  digitalWrite(FRONT_IN1, HIGH);
  digitalWrite(FRONT_IN2, LOW);
  analogWrite(FRONT_ENA, 220);
  delay(2500);
  
  Serial.println("[FRONT-LEFT] BACKWARD (PWM=220)");
  digitalWrite(FRONT_IN1, LOW);
  digitalWrite(FRONT_IN2, HIGH);
  analogWrite(FRONT_ENA, 220);
  delay(2500);
  
  Serial.println("[FRONT-LEFT] STOP");
  digitalWrite(FRONT_IN1, LOW);
  digitalWrite(FRONT_IN2, LOW);
  analogWrite(FRONT_ENA, 0);
  delay(1000);
  
  // ===== FRONT RIGHT MOTOR =====
  Serial.println("\n[FRONT-RIGHT] FORWARD (PWM=220)");
  digitalWrite(FRONT_IN3, HIGH);
  digitalWrite(FRONT_IN4, LOW);
  analogWrite(FRONT_ENB, 220);
  delay(2500);
  
  Serial.println("[FRONT-RIGHT] BACKWARD (PWM=220)");
  digitalWrite(FRONT_IN3, LOW);
  digitalWrite(FRONT_IN4, HIGH);
  analogWrite(FRONT_ENB, 220);
  delay(2500);
  
  Serial.println("[FRONT-RIGHT] STOP");
  digitalWrite(FRONT_IN3, LOW);
  digitalWrite(FRONT_IN4, LOW);
  analogWrite(FRONT_ENB, 0);
  delay(1000);
  
  // ===== BACK LEFT MOTOR =====
  Serial.println("\n[BACK-LEFT] FORWARD (PWM=220)");
  digitalWrite(BACK_IN1, HIGH);
  digitalWrite(BACK_IN2, LOW);
  analogWrite(BACK_ENA, 220);
  delay(2500);
  
  Serial.println("[BACK-LEFT] BACKWARD (PWM=220)");
  digitalWrite(BACK_IN1, LOW);
  digitalWrite(BACK_IN2, HIGH);
  analogWrite(BACK_ENA, 220);
  delay(2500);
  
  Serial.println("[BACK-LEFT] STOP");
  digitalWrite(BACK_IN1, LOW);
  digitalWrite(BACK_IN2, LOW);
  analogWrite(BACK_ENA, 0);
  delay(1000);
  
  // ===== BACK RIGHT MOTOR =====
  Serial.println("\n[BACK-RIGHT] FORWARD (PWM=220)");
  digitalWrite(BACK_IN3, HIGH);
  digitalWrite(BACK_IN4, LOW);
  analogWrite(BACK_ENB, 220);
  delay(2500);
  
  Serial.println("[BACK-RIGHT] BACKWARD (PWM=220)");
  digitalWrite(BACK_IN3, LOW);
  digitalWrite(BACK_IN4, HIGH);
  analogWrite(BACK_ENB, 220);
  delay(2500);
  
  Serial.println("[BACK-RIGHT] STOP");
  digitalWrite(BACK_IN3, LOW);
  digitalWrite(BACK_IN4, LOW);
  analogWrite(BACK_ENB, 0);
  delay(1000);
  
  Serial.println("\n=== CYCLE COMPLETE ===\n");
  delay(2000);
}

void stopAllMotors() {
  // Stop front motors
  digitalWrite(FRONT_IN1, LOW);
  digitalWrite(FRONT_IN2, LOW);
  analogWrite(FRONT_ENA, 0);
  
  digitalWrite(FRONT_IN3, LOW);
  digitalWrite(FRONT_IN4, LOW);
  analogWrite(FRONT_ENB, 0);
  
  // Stop back motors
  digitalWrite(BACK_IN1, LOW);
  digitalWrite(BACK_IN2, LOW);
  analogWrite(BACK_ENA, 0);
  
  digitalWrite(BACK_IN3, LOW);
  digitalWrite(BACK_IN4, LOW);
  analogWrite(BACK_ENB, 0);
}
