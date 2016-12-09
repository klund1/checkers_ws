#define J0 2
#define J1 3
#define J2 4
#define J3 5
#define EM_PIN 6

#include <ros.h>
#include <std_msgs/Bool.h>
#include <checkers_arm/JointAngles.h>

#include <Servo.h>
Servo joint0, joint1, joint2, joint3;

void setElectroMagnet(bool on_off) {
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
  if (a2 >= -90 && a2 <= 90) {
    float write_angle = map(a2,-90,90,0,180);
    joint2.write(write_angle);
  }
  if (a3 >= -90 && a3 <= 90) {
    float write_angle = map(a3,-90,90,0,160);
    joint3.write(write_angle);
  }
}

void setup() {
  Serial.begin(9600);
  
  pinMode(EM_PIN, OUTPUT);
  
  joint0.attach(J0);
  joint1.attach(J1);
  joint2.attach(J2);
  joint3.attach(J3);
  
  setArmPosition(0,45,-45,0);
  setElectroMagnet(false);
}

long read_in;
bool EM_state;

void loop() {
  if (Serial.available()) {
    EM_state = (Serial.parseInt() != 0);
    setElectroMagnet(EM_state);
  }
}
