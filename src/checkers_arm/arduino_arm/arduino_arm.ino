#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
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

std_msgs::String str_msg;
ros::Publisher event_pub("event", &str_msg);

void setup() {
  Serial.begin(19200);
  initServos();
  initElectromagnet();
  nh.subscribe(emSub);
  nh.subscribe(armSub);
  nh.advertise(event_pub);
}

bool last_val = false;
bool current_val = false;

void loop() {
  current_val = button_val();
  if (current_val && !last_val) {
    str_msg.data = "HUMAN:FINISHED";
    event_pub.publish(&str_msg);
  }
  last_val = current_val;
  nh.spinOnce();
  delay(100);
}
