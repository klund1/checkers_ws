#ifndef CHECKERS_H
#define CHECKERS_H

#include <cstdint>
#include <vector>
using std::vector;

#define PLY_DEFAULT 5
#define GAMMA_DEFAULT 0.99

const float POS_SCORE[8][8] = {
  {4, 0, 4, 0, 4, 0, 4, 0},
  {0, 3, 0, 3, 0, 3, 0, 4},
  {4, 0, 2, 0, 2, 0, 3, 0},
  {0, 3, 0, 1, 0, 2, 0, 4},
  {4, 0, 2, 0, 1, 0, 3, 0},
  {0, 3, 0, 2, 0, 2, 0, 4},
  {4, 0, 3, 0, 3, 0, 3, 0},
  {0, 4, 0, 4, 0, 4, 0, 4} };


enum GameState {
  ONGOING,
  WHITE_WON,
  BLACK_WON,
  STALEMATE
};

enum PieceType {
  EMPTY,
  WHITE,
  BLACK
};

enum AiType {
  RANDOM,
  GAMETREE
};

struct piece {
  PieceType type;
  int num;
  bool king;
};

struct pos {
  int8_t i,j;
};

typedef vector<pos> move_t;

bool validPos(pos p);

class Checkers {
 public:
  Checkers();
  void resetBoard();
  void clearBoard();
  void resetWinCheck();
  void resetHopCheck();
  void printBoard() const;

  void setPly(int ply);

  bool addPiece(pos p, PieceType color, bool king);
  bool applyMove(move_t move);

  void humanMove(PieceType color);
  move_t aiMove(PieceType color, AiType ai);
  void printMoves(PieceType color) const;
  GameState state() const;
  float score(PieceType color);

 private:
  piece board_[8][8];
  pos white_pos_[12];
  pos black_pos_[12];
  int ply_;
  float gamma_;

  piece& getPiece(int i, int j);
  const piece& getPiece(int i, int j) const;
  piece& getPiece(pos p);
  const piece& getPiece(pos p) const;

  move_t aiMoveRandom(PieceType color);
  move_t aiMoveGameTree(PieceType color);
  move_t minimax(int depth, float &val, float alpha, float beta, bool max_player, PieceType color);

  bool move(pos start, pos end);
  bool remove(pos p);
  bool undoMove(move_t move, bool prev_kings[8][8]);

  int numPieces(PieceType color) const;
  int numKings(PieceType color) const;
  float posScore(PieceType color) const;

  bool canPush(pos start, pos end) const;
  bool canHop(pos start, pos end) const;
  bool canHop(pos start) const;
  bool canHop(PieceType color) const;

  vector<move_t> getMoves(PieceType color) const;
  vector<move_t> getHops(pos start) const;
  void hopDFS(pos start, PieceType hop_color, bool king, move_t &cur_move, vector<move_t> &moves, bool hopped[8][8]) const;
};

#endif //CHECKERSAI_H
