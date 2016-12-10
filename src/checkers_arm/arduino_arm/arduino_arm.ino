#include <ros.h>
#include <std_msgs/Bool.h>
#include <checkers_arm/JointAngles.h>

ros::NodeHandle nh;

void emCb( const std_msgs::Bool& msg ) {
  setElectromagnet(msg.data);
}

void armCb( const checkers_arm::JointAngles& msg ) {
  setArmPosition(msg.j0, msg.j1, msg.j2, msg.j3);
}

ros::Subscriber<std_msgs::Bool> emSub("electromagnet", emCb);
ros::Subscriber<checkers_arm::JointAngles> armSub("arm_position", armCb);

void setup() {
  Serial.begin(57600);
  initServos();
  initElectromagnet();
  nh.subscribe(emSub);
  nh.subscribe(armSub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}
