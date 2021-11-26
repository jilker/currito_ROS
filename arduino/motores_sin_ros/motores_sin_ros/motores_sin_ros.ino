int MOTOR1_DIRECCION_PIN = 12;
int MOTOR2_DIRECCION_PIN = 13;
int MOTOR1_VELOC_PIN = 10;
int MOTOR2_VELOC_PIN = 11;
int para = 0;
int velocidad = 0;
void setup() {
  pinMode(MOTOR1_DIRECCION_PIN, OUTPUT);
  pinMode(MOTOR2_DIRECCION_PIN, OUTPUT);
  pinMode(MOTOR1_VELOC_PIN, OUTPUT);
  pinMode(MOTOR2_VELOC_PIN, OUTPUT);
}
void loop() {
  digitalWrite(MOTOR1_DIRECCION_PIN, HIGH);
  digitalWrite(MOTOR2_DIRECCION_PIN, HIGH);
  analogWrite(MOTOR1_VELOC_PIN, velocidad);
  analogWrite(MOTOR2_VELOC_PIN, velocidad);
  delay(2000);
  analogWrite(MOTOR1_VELOC_PIN, para);
  analogWrite(MOTOR2_VELOC_PIN, para);
  delay(2000);
  digitalWrite(MOTOR2_DIRECCION_PIN, LOW);
  digitalWrite(MOTOR1_DIRECCION_PIN, LOW);
  analogWrite(MOTOR1_VELOC_PIN, velocidad);
  analogWrite(MOTOR2_VELOC_PIN, velocidad);
  delay(2000);
  analogWrite(MOTOR1_VELOC_PIN, para);
  analogWrite(MOTOR2_VELOC_PIN, para);
  delay(2000);
}
