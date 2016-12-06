#include "checkers.hpp"
#include "ros/ros.h"
#include "checkers_ai/AddPiece.h"
#include "std_msgs/String.h"

#include <string>
#include <iostream>
using std::string;

// Global variables
Checkers my_checkers;
PieceType my_color;
string last_state = "";

// Publishers
ros::Publisher eventPub; 
ros::Publisher movePub;

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
    if (msg->remove) {
      my_checkers.remove(p);
    }
    else {
      PieceType piece_color;
      piece_color = msg->color ? WHITE : BLACK;
      my_checkers.addPiece(p,piece_color,msg->king);
    }
  }
}

void stateCallback(const std_msgs::String::ConstPtr& msg) {
  string state = msg->data;
  if (state.compare(last_state) != 0 && state.compare("AI_COMPUTE") == 0) {
    printf("----- START MOVE -----\n");
    my_checkers.printBoard();
    move_t m = my_checkers.aiMove(my_color, GAMETREE);
    my_checkers.printBoard();
    std::ostringstream stream;
    for (auto p : m) {
      stream << "(" << static_cast<int>(p.i) << "," << static_cast<int>(p.j) << ") ";
    }
    std_msgs::String move_msg;
    move_msg.data = stream.str();
    movePub.publish(move_msg);

    std_msgs::String event_msg;
    event_msg.data = "AI:FINISHED";
    eventPub.publish(event_msg);
  }
  last_state = state;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "checkers_ai");

  ros::NodeHandle n;

  bool color_bool;
  n.param("color", color_bool, true);
  my_color = color_bool ? WHITE : BLACK;

  int ply;
  n.param("ply", ply, 10);
  my_checkers.setPly(ply);

  int queue_size = 10;
  ros::Subscriber add_sub  = n.subscribe("add_piece", queue_size, addPieceCallback);
  ros::Subscriber state_sub = n.subscribe("state", queue_size, stateCallback);

  eventPub = n.advertise<std_msgs::String>("event",10);
  movePub = n.advertise<std_msgs::String>("move_piece",10);

  ros::spin();

  return 0;
}
