#include "checkers.hpp"
#include <string>
#include "ros/ros.h"
#include "checkers_ai/AddPiece.h"
#include "std_msgs/String.h"

using std::string;

// Global variables
Checkers my_checkers;
string last_state = "";


void addPieceCallback(const checkers_ai::AddPiece::ConstPtr& msg) {
  if (msg->clear) {
    my_checkers.clearBoard();
  }
  if (msg->reset) {
    my_checkers.resetBoard();
    return;
  }

  pos p{msg->i,msg->j};

  if (validPos(p)) {
    if (msg->color) {
      my_checkers.addPiece(p,WHITE,msg->king);
    } else {
      my_checkers.addPiece(p,BLACK,msg->king);
    }
  }
}

void stateCallback(const std_msgs::String::ConstPtr& msg) {
  string state = msg->data;
  if (state.compare(last_state) != 0 && state.compare("AI_COMPUTE") == 0) {
  //TODO
  }
  last_state = state;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "checkers_ai");

  ros::NodeHandle n;
  int queue_size = 10;
  ros::Subscriber add_sub  = n.subscribe("add_piece", queue_size, addPieceCallback);
  ros::Subscriber state_sub = n.subscribe("state", queue_size, stateCallback);

  ros::spin();

  return 0;
}
