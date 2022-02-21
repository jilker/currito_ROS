#include <Servo.h>

  Servo rueda_izq_;
  Servo rueda_der_;

#define MAX 65535 // (2^16 - 1)
int ms = 50000;

int MOTOR1_DIRECCION_PIN = 12;
int MOTOR2_DIRECCION_PIN = 13;
int MOTOR1_VELOC_PIN = 10;
int MOTOR2_VELOC_PIN = 11;


void setup() {
  pinMode(MOTOR1_DIRECCION_PIN, OUTPUT);
  pinMode(MOTOR2_DIRECCION_PIN, OUTPUT);

    rueda_izq_.attach( MOTOR1_VELOC_PIN,   0,   MAX);
    rueda_der_.attach( MOTOR2_VELOC_PIN,   0,   MAX);
}



void loop() {
  digitalWrite(MOTOR1_DIRECCION_PIN, HIGH);
  digitalWrite(MOTOR2_DIRECCION_PIN, HIGH);
//rueda_izq_.write(180);
//rueda_der_.write(180);
rueda_izq_.writeMicroseconds(ms);
rueda_der_.writeMicroseconds(ms);

//  delay(5000);
//rueda_izq_.write(100);
//rueda_der_.write(100);
//  delay(5000);
//rueda_izq_.write(0);
//rueda_der_.write(0);
//  delay(5000);
//
//digitalWrite(MOTOR1_DIRECCION_PIN, LOW);
//digitalWrite(MOTOR2_DIRECCION_PIN, LOW);
//
//rueda_izq_.write(180);
//rueda_der_.write(180);
//  delay(5000);
//rueda_izq_.write(100);
//rueda_der_.write(100);
//  delay(5000);
//rueda_izq_.write(0);
//rueda_der_.write(0);
//  delay(5000);
}
