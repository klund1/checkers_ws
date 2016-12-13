// Low level communication to the arm and electromagnet
#define J0 2
#define J1 3
#define J2 4
#define J3 5
#define EM_PIN 6
#define BUTTON_PIN 7

#include <Servo.h>
Servo joint0, joint1, joint2, joint3;

void setElectromagnet(bool on_off) {
  digitalWrite(EM_PIN,on_off);
}

void setArmPosition(float a0, float a1, float a2, float a3) {
  if (a0 >= -90 && a0 <= 90) {
    float write_angle = map(a0,-90,90,0,180);
    joint0.write(write_angle);
  }
  if (a1 >= 0 && a1 <= 180) {
    float write_angle = map(a1,0,180,180,0);
    joint1.write(write_angle);
  }
  if (a2 >= 0 && a2 <= 180) {
    float write_angle = map(a2,0,180,180,0);
    joint2.write(write_angle);
  }
  if (a3 >= -90 && a3 <= 90) {
    float write_angle = map(a3,-90,90,160,0);
    joint3.write(write_angle);
  }
}

void initServos() {
  joint0.attach(J0);
  joint1.attach(J1);
  joint2.attach(J2);
  joint3.attach(J3);
  //move arm to default position
  setArmPosition(0,90,0,0);
}

void initElectromagnet() {
  pinMode(EM_PIN, OUTPUT);
  setElectromagnet(false);
}

void initButton() {
  pinMode(BUTTON_PIN, INPUT);
}

bool button_val() {
  return digitalRead(BUTTON_PIN);
}
