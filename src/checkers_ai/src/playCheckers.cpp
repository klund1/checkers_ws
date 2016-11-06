#include "checkers.hpp"
#include <cstdio>

int main() {
  printf("Welcome to checkers!\n");
  Checkers myCheckers;
  //myCheckers.resetHopCheck();
  myCheckers.printBoard();
  myCheckers.setPly(8);

  while(myCheckers.state() == ONGOING){
    myCheckers.printMoves(WHITE);
    myCheckers.aiMove(WHITE,GAMETREE);
    myCheckers.printBoard();
    if (myCheckers.state() != ONGOING) {
      continue;
    }
    myCheckers.printMoves(BLACK);
    myCheckers.humanMove(BLACK);
    myCheckers.printBoard();
  }

  switch (myCheckers.state()) {
    case WHITE_WON:
      printf("WHITE wins!\n");
      break;
    case BLACK_WON:
      printf("BLACK wins!\n");
      break;
    case STALEMATE:
      printf("STALEMATE!\n");
      break;
    default:
      printf("UNKNOWN EXIT STATE...\n");
  }

  return 0;
}
