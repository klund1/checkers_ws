#include "checkers.hpp"
#include "ros/ros.h"
#include "checkers_ai/AddPiece.h"
#include "checkers_ai/MovePiece.h"
#include "checkers_ai/GetMove.h"

Checkers myCheckers;

void addPieceCallback(const checkers_ai::AddPiece::ConstPtr& msg) {
  if (msg->clear) {
    myCheckers.clearBoard();
  }
  if (msg->reset) {
    myCheckers.resetBoard();
    return;
  }

  pos p{msg->i,msg->j};

  if (validPos(p)) {
    if (msg->color) {
      myCheckers.addPiece(p,WHITE,msg->king);
    } else {
      myCheckers.addPiece(p,BLACK,msg->king);
    }
  }
}

void movePieceCallback(const checkers_ai::MovePiece::ConstPtr& msg) {
  pos p{msg->start_i, msg->start_j};
  pos q{msg->end_i, msg->end_j};
  move_t m;
  m.push_back(p);
  m.push_back(q);

  myCheckers.applyMove(m);
}

void getMoveService(checkers_ai::GetMove::Request &req,
                    checkers_ai::GetMove::Response &res) {
  myCheckers.setPly(req.ply);
  PieceType color = req.color ? WHITE : BLACK;
  move_t m = myCheckers.aiMove(color, GAMETREE);
  
  char buff[100];
  for (auto p : m) {
    sprintf(buff,"(%d,%d) ",p.i,p.j);
  }
  res.move = buff;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "checkers_ai");

  ros::NodeHandle n;
  int queue_size = 10;
  ros::Subscriber add_sub  = n.subscribe("add_piece", queue_size, addPieceCallback);
  ros::Subscriber move_sub = n.subscribe("move_piece", queue_size, movePieceCallback);

  ros::spin();

  return 0;
}
