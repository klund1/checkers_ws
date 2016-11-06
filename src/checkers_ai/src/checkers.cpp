#include "checkers.hpp"
#include <cstdio>
#include <string>
#include <cstdlib>
#include <cmath>
#include <random>
#include <iterator>
#include <ctime>
#include <limits>
#include <algorithm>

#define F_INF std::numeric_limits<float>::infinity()

using std::string;
char pieceChar(piece p);
int get0to7(string msg);
bool validPos(pos p);

Checkers::Checkers() : ply_{PLY_DEFAULT}, gamma_{GAMMA_DEFAULT} {
  this->resetBoard();
  srand(time(NULL));
}

void Checkers::setPly(int ply) {
  this->ply_ = ply;
}

void Checkers::resetBoard() {
  this->clearBoard();

  //reset white pieces
  for(int n = 0; n < 12; ++n) {
    int i = n/4;
    int j = 2*(n%4) + (n/4)%2;
    piece &pc = this->getPiece(i,j);
    pc.type = WHITE;
    pc.num = n;
    pc.king = false;
    this->white_pos_[n].i = i;
    this->white_pos_[n].j = j;
  }
  //reset black pieces
  for(int n = 0; n < 12; ++n) {
    int i = n/4 + 5;
    int j = 2*(n%4) + (1 - (n/4)%2);
    piece &pc = this->getPiece(i,j);
    pc.type = BLACK;
    pc.num = n;
    pc.king = false;
    this->black_pos_[n].i = i;
    this->black_pos_[n].j = j;
  }
}

void Checkers::clearBoard() {
  for (int i=0; i < 8; ++i) {
    for (int j=0; j < 8; ++j) {
      piece &pc = this->getPiece(i,j);
      pc.type = EMPTY;
      pc.king = false;
      pc.num = -1;
    }
  }
  for (int n = 0; n < 12; ++n) {
    this->white_pos_[n].i = -1;
    this->black_pos_[n].i = -1;
  }
}

void Checkers::resetWinCheck() {
  this->clearBoard();

  pos p;

  p.i = 4;
  p.j = 4;
  this->addPiece(p,WHITE,false);

  p.i = 5;
  p.j = 5;
  this->addPiece(p,BLACK,false);
}

void Checkers::resetHopCheck() {
  this->clearBoard();

  pos p;

  p.i = 2;
  p.j = 2;
  this->addPiece(p,WHITE,false);

  p.i = 3;
  p.j = 1;
  this->addPiece(p,BLACK,false);

  p.i = 3;
  p.j = 3;
  this->addPiece(p,BLACK,false);

  p.i = 5;
  p.j = 3;
  this->addPiece(p,BLACK,false);

  p.i = 5;
  p.j = 5;
  this->addPiece(p,BLACK,false);

  p.i = 5;
  p.j = 1;
  this->addPiece(p,BLACK,false);

  p.i = 1;
  p.j = 1;
  this->addPiece(p,BLACK,false);
}

GameState Checkers::state() const{
  bool white_pieces, black_pieces = false;
  for (int n = 0; n < 12; ++n) {
    if (validPos(white_pos_[n]) ){
      white_pieces = true;
      break;
    }
  }

  for (int n = 0; n < 12; ++n) {
    if (validPos(black_pos_[n]) ){
      black_pieces = true;
      break;
    }
  }

  if (!white_pieces && black_pieces) {
    return BLACK_WON;
  }
  else if (white_pieces && !black_pieces) {
    return WHITE_WON;
  }
  else if (!white_pieces && !black_pieces) {
    return STALEMATE;
  }
  else {
    return ONGOING;
  }
}

void Checkers::humanMove(PieceType color) {
  pos start_pos, end_pos, hop_pos;
  bool push, hop, must_hop, valid_move = false;

  switch(color){
    case BLACK: printf("Black player's turn!\n"); break;
    case WHITE: printf("White player's turn!\n"); break;
    default: printf("Unknown player type!\n"); return;
  }

  must_hop = this->canHop(color);

  while (!valid_move) {
    printf("Select a piece\n");
    start_pos.i = get0to7("    i: ");
    start_pos.j = get0to7("    j: ");
    if (this->getPiece(start_pos).type != color) {
      printf("You don't have a piece at (%d,%d)!\n",start_pos.i,start_pos.j);
      continue;
    }

    printf("Select a destination\n");
    end_pos.i = get0to7("    i: ");
    end_pos.j = get0to7("    j: ");
    if (this->getPiece(end_pos).type != EMPTY) {
      printf("The destination is not empty!\n");
      continue;
    }

    push = this->canPush(start_pos,end_pos);
    hop  = this->canHop(start_pos,end_pos);
    valid_move = (!must_hop && push) || hop;

    if (must_hop && push) {
      printf("You must hop a piece!\n");
    }

    if (!valid_move) {
      printf("Invalid move!\n");
    }
  }

  if (push) {
    this->move(start_pos, end_pos);
  }
  else if (hop) {
    hop_pos.i = (start_pos.i+end_pos.i)/2;
    hop_pos.j = (start_pos.j+end_pos.j)/2;
    this->move(start_pos, end_pos);
    this->remove(hop_pos);

    this->printBoard();
    printf("HOPPED!\n");

    start_pos = end_pos;
  }

  while (this->canHop(start_pos)) {
    printf("Hop again!\n");
    hop = false;

    while (!hop) {
      printf("Select a destination\n");
      end_pos.i = get0to7("    i: ");
      end_pos.j = get0to7("    j: ");
      if (this->getPiece(end_pos).type != EMPTY) {
        printf("The destination is not empty!\n");
        continue;
      }

      hop = this->canHop(start_pos, end_pos);
      if (!hop) {
        printf("Not a valid hop!\n");
      }
    }
    hop_pos.i = (start_pos.i+end_pos.i)/2;
    hop_pos.j = (start_pos.j+end_pos.j)/2;
    this->move(start_pos, end_pos);
    this->remove(hop_pos);

    this->printBoard();
    printf("HOPPED!\n");

    start_pos = end_pos;
  }

}

int get0to7(string msg) {
  char c;
  int ret_val = -1;

  printf("%s", msg.c_str());
  scanf("%d", &ret_val);
  while (ret_val < 0 || ret_val > 7) {
    printf("Must be between 0 and 7! Try again: ");
    while ((c = getchar()) != '\n' && c != EOF) { };
    scanf("%d", &ret_val);
  }
  return ret_val;
}

void Checkers::aiMove(PieceType color, AiType ai) {
  move_t m;

  switch(color){
    case BLACK: printf("Black AI's turn!\n"); break;
    case WHITE: printf("White AI's turn!\n"); break;
    default: printf("Unknown AI type!\n"); return;
  }

  switch (ai) {
    case RANDOM:
      m = this->aiMoveRandom(color);
      break;
    case GAMETREE:
      m = this->aiMoveGameTree(color);
      break;
  }

  printf("AI move:");
  for (auto p : m) {
    printf(" (%d,%d)",p.i,p.j);
  }
  printf("\n");
  this->applyMove(m);
}

move_t Checkers::aiMoveRandom(PieceType color) {
  vector<move_t> moves = getMoves(color);
  auto it = moves.begin();
  std::advance(it, std::rand() % moves.size());
  return *it;
}

move_t Checkers::aiMoveGameTree(PieceType color) {
  float val;
  move_t m;
  m = this->minimax(this->ply_,val,-F_INF,F_INF,true,color) ;
  printf("Gametree returned with a value of %f\n",val);
  return m;
}

move_t Checkers::minimax(int depth,
                         float &val,
                         float alpha,
                         float beta,
                         bool max_player,
                         PieceType color) {
  vector<move_t> moves;
  move_t best_move,cur_move;
  float best_val;
  PieceType opp_color;
  bool kings[8][8];

  switch (color) {
    case BLACK:
      opp_color = WHITE;
      break;
    case WHITE:
      opp_color = BLACK;
      break;
    case EMPTY:
      return best_move; //empty
  }

  if (this->state() != ONGOING || depth == 0) {
    if (max_player) {
      val = pow(this->gamma_, this->ply_ - depth) * this->score(color);
    }
    else {
      val = pow(this->gamma_, this->ply_ - depth) * this->score(opp_color);
    }
    return best_move; //this is empty
  }

  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 8; ++j) {
      pos cur_pos = {i,j};
      kings[i][j] = this->getPiece(cur_pos).king;
    }
  }

  moves = this->getMoves(color);
  std::random_shuffle(moves.begin(),moves.end());

  if (max_player) {
    best_val = -F_INF;
    for (auto m : moves) {
      //apply move, recurse, and then undo move
      this->applyMove(m);
      this->minimax(depth-1,val,alpha,beta,!max_player,opp_color);
      this->undoMove(m, kings);

      //update best_val, best_move, and alpha
      if (val > best_val) {
        best_move = m;
      }
      best_val = fmax(best_val, val);
      alpha = fmax(alpha, best_val);

      //check for alpha cutoff
      if (beta <= alpha) {
        val = best_val;
        return best_move;
      }
    }
  }
  else {
    best_val = F_INF;
    for (auto m : moves) {
      //apply move, recurse, and then undo move
      this->applyMove(m);
      this->minimax(depth-1,val,alpha,beta,!max_player,opp_color);
      this->undoMove(m, kings);

      //update best_val, best_move, and beta
      if (val < best_val) {
        best_move = m;
      }
      best_val = fmin(best_val, val);
      beta = fmin(beta, best_val);

      //check for beta cutoff
      if (beta <= alpha) {
        val = best_val;
        return best_move;
      }
    }
  }

  val = best_val;
  return best_move;
}

float Checkers::score(PieceType color) {
  PieceType opp_color;
  float my_piece_score, opp_piece_score, piece_score,
        my_pos_score, opp_pos_score, pos_score;
  switch (color) {
    case WHITE:
      opp_color = BLACK;
      break;
    case BLACK:
      opp_color = WHITE;
      break;
    case EMPTY:
      return 0;
  }

  my_piece_score = this->numPieces(color) + this->numKings(color);
  opp_piece_score = this->numPieces(opp_color) + this->numKings(opp_color);

  piece_score = (my_piece_score - opp_piece_score) / (my_piece_score + opp_piece_score);

  my_pos_score = this->posScore(color);
  opp_pos_score = this->posScore(opp_color);

  pos_score = (my_pos_score - opp_pos_score) / (my_pos_score + opp_pos_score);

  return piece_score + 0.1*pos_score;
}

int Checkers::numPieces(PieceType color) const {
  const pos* pos_list;
  int num_pieces = 0;

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    case EMPTY:
      return 0;
  }

  for (int n = 0; n < 12; ++n) {
    if (validPos(pos_list[n])) {
      num_pieces++;
    }
  }

  return num_pieces;
}

int Checkers::numKings(PieceType color) const {
  const pos* pos_list;
  int num_kings = 0;

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    case EMPTY:
      return 0;
  }

  for (int n = 0; n < 12; ++n) {
    if (validPos(pos_list[n]) && this->getPiece(pos_list[n]).king) {
      num_kings++;
    }
  }

  return num_kings;
}

float Checkers::posScore(PieceType color) const {
  const pos* pos_list;
  pos p;
  float score = 0;

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    case EMPTY:
      return 0;
  }

  for (int n = 0; n < 12; ++n) {
    p = pos_list[n];
    if (validPos(p)) {
      score += POS_SCORE[p.i][p.j];
    }
  }

  return score;
}


bool Checkers::applyMove(move_t m) {
  pos hop;
  pos last_p = {-1,-1};
  for (auto p : m) {
    if (!validPos(p)) {
      return false;
    }
    if (validPos(last_p)) {
      if (canPush(last_p,p)) {
        this->move(last_p,p);
      }
      else if (canHop(last_p,p)) {
        hop.i = (p.i + last_p.i)/2;
        hop.j = (p.j + last_p.j)/2;
        this->move(last_p,p);
        this->remove(hop);
      }
      else {
        return false;
      }
    }
    last_p = p;
  }

  return true;
}

bool Checkers::undoMove(move_t m, bool prev_kings[8][8]) {
  pos hop;
  pos last_p = {-1,-1};
  PieceType color, opp_color;

  std::reverse(m.begin(), m.end());

  color = this->getPiece(m[0]).type;
  switch (color) {
    case WHITE:
      opp_color = BLACK;
      break;
    case BLACK:
      opp_color = WHITE;
      break;
    case EMPTY:
      printf("Unable to undo move! (empty)\n");
      return false;
  }

  for (auto p : m) {
    if (!validPos(p)) {
      printf("Unable to undo move! (bad position in move)\n");
      return false;
    }

    if (validPos(last_p)) {
      if (abs(p.i-last_p.i) == 1 && abs(p.j-last_p.j) == 1){
        this->move(last_p,p);
      }
      else if (abs(p.i-last_p.i) == 2 && abs(p.j-last_p.j) == 2){
        hop.i = (p.i+last_p.i)/2;
        hop.j = (p.j+last_p.j)/2;

        this->move(last_p,p);
        this->addPiece(hop,opp_color,prev_kings[hop.i][hop.j]);
      }
      else {
        printf("Unable to undo move! (can't hop or push (%d,%d) to (%d,%d))\n",last_p.i,last_p.j,p.i,p.j);
        return false;
      }
    }
    last_p = p;
  }
  this->getPiece(last_p).king = prev_kings[last_p.i][last_p.j];

  return true;
}

piece& Checkers::getPiece(int i, int j) {
  return this->board_[i][j];
}

const piece& Checkers::getPiece(int i, int j) const {
  return this->board_[i][j];
}

piece& Checkers::getPiece(pos p) {
  return this->board_[p.i][p.j];
}

const piece& Checkers::getPiece(pos p) const {
  return this->board_[p.i][p.j];
}

bool Checkers::canPush(pos start, pos end) const {
  if (!validPos(start) || !validPos(end)) {
    return false;
  }

  const piece &p_mov = this->getPiece(start);
  if (p_mov.type == EMPTY) {
    return false;
  }

  const piece &p_end = this->getPiece(end);
  if (p_end.type != EMPTY) {
    return false;
  }

  if (p_mov.type == WHITE || p_mov.king) {
    if (end.i == start.i + 1 && abs(start.j-end.j) == 1) {
      return true;
    }
  }
  if (p_mov.type == BLACK || p_mov.king) {
    if (end.i == start.i - 1 && abs(start.j-end.j) == 1) {
      return true;
    }
  }

  return false;
}

bool Checkers::canHop(pos start, pos end) const{
  pos hop;
  if (!validPos(start) || !validPos(end)) {
    return false;
  }

  const piece &p_mov = this->getPiece(start);
  if (p_mov.type == EMPTY) {
    return false;
  }

  const piece &p_end = this->getPiece(end);
  if (p_end.type != EMPTY) {
    return false;
  }

  if (p_mov.type == WHITE || p_mov.king) {
    if (end.i == start.i + 2 && abs(start.j-end.j) == 2) {
      hop.i = (start.i+end.i)/2;
      hop.j = (start.j+end.j)/2;
      const piece &p_hop = this->getPiece(hop);
      if (p_hop.type != EMPTY && p_hop.type != p_mov.type) {
        return true;
      }
    }
  }

  if (p_mov.type == BLACK || p_mov.king) {
    if (end.i == start.i - 2 && abs(start.j-end.j) == 2) {
      hop.i = (start.i+end.i)/2;
      hop.j = (start.j+end.j)/2;
      const piece &p_hop = this->getPiece(hop);
      if (p_hop.type != EMPTY && p_hop.type != p_mov.type) {
        return true;
      }
    }
  }

  return false;
}

bool Checkers::canHop(pos start) const{
  pos end;
  if (!validPos(start)) {
    return false;
  }

  for (int i = start.i - 2; i <= start.i + 2; i += 4) {
    for (int j = start.j - 2; j <= start.j + 2; j += 4) {
      end.i = i;
      end.j = j;
      if (this->canHop(start,end)) {
        return true;
      }
    }
  }

  return false;
}

bool Checkers::move(pos start, pos end) {
  if (!validPos(start) || !validPos(end)) {
    return false;
  }
  piece& p_start = this->getPiece(start);
  piece& p_end = this->getPiece(end);
  if (p_end.type != EMPTY) {
    return false;
  }

  switch (p_start.type) {
    case WHITE:
      white_pos_[p_start.num].i = end.i;
      white_pos_[p_start.num].j = end.j;
      break;
    case BLACK:
      black_pos_[p_start.num].i = end.i;
      black_pos_[p_start.num].j = end.j;
      break;
    default:
      return false;
  }
  p_end = p_start;
  p_start.type = EMPTY;

  if (!p_end.king && p_end.type == WHITE && end.i == 7) {
    p_end.king = true;
  }

  if (!p_end.king && p_end.type == BLACK && end.i == 0) {
    p_end.king = true;
  }

  return true;
}

bool Checkers::remove(pos p) {
  if (!validPos(p)) {
    return false;
  }

  piece &p_rem = this->getPiece(p);

  switch (p_rem.type) {
    case WHITE:
      white_pos_[p_rem.num].i = -1;
      break;
    case BLACK:
      black_pos_[p_rem.num].i = -1;
      break;
    default:
      return false;
  }
  p_rem.type = EMPTY;

  return true;
}

bool Checkers::addPiece(pos p, PieceType color, bool king) {
  pos* pos_list;

  if (!validPos(p) || this->getPiece(p).type != EMPTY) {
    return false;
  }

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    default:
      return true;
  }

  for (int n = 0; n < 12; ++n) {
    if (!validPos(pos_list[n])) {
      pos_list[n] = p;
      piece &pc = this->getPiece(p);
      pc.type = color;
      pc.num = n;
      pc.king = king;
      return true;
    }
  }

  return false;
}

vector<move_t> Checkers::getMoves(PieceType color) const {
  vector<move_t> moves;
  const pos* pos_list;
  pos p,q;

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    default:
      return moves;
  }

  //check hops from each position
  for (int n = 0; n < 12; ++n) {
    p = pos_list[n];

    vector<move_t> hops = this->getHops(p);
    moves.insert( moves.end(), hops.begin(), hops.end());
  }

  if (moves.size() == 0) {
    for (int n = 0; n < 12; ++n) {
      p = pos_list[n];

      for (int i = p.i - 1; i <= p.i + 1; i += 2) {
        for (int j = p.j - 1; j <= p.j + 1; j += 2) {
          q.i = i;
          q.j = j;
          if (this->canPush(p,q)) {
            move_t m;
            m.push_back(p);
            m.push_back(q);
            moves.push_back(m);
          }
        }
      }
    }

  }
  return moves;
}

bool Checkers::canHop(PieceType color) const {
  const pos* pos_list;
  pos p;

  switch (color) {
    case WHITE:
      pos_list = this->white_pos_;
      break;
    case BLACK:
      pos_list = this->black_pos_;
      break;
    default:
      return false;
  }

  //check hops from each position
  for (int n = 0; n < 12; ++n) {
    p = pos_list[n];
    if (this->canHop(p)) {
      return true;
    }
  }
  return false;
}

vector<move_t> Checkers::getHops(pos start) const {
  vector<move_t> hops;
  if (!validPos(start)) {
    return hops;
  }

  PieceType color = this->getPiece(start).type;
  PieceType hop_color;
  switch (color) {
    case EMPTY:
      return hops;
    case WHITE:
      hop_color = BLACK;
      break;
    case BLACK:
      hop_color = WHITE;
      break;
  }

  bool king = this->getPiece(start).king;

  //initially no pieces have been hopped
  //but, to allow our piece to make a cycle, we set the current position to true
  bool hopped[8][8] = {{0}};
  hopped[start.i][start.j] = 1;

  //all moves must start with our current position
  move_t cur_move;
  cur_move.push_back(start);

  hopDFS(start,hop_color,king,cur_move,hops,hopped);
  return hops;
}

void Checkers::hopDFS(pos start,
                      PieceType hop_color,
                      bool king,
                      move_t &cur_move,
                      vector<move_t> &moves,
                      bool hopped[8][8]) const {

  pos pos_end, pos_hop;
  bool added_move = false;
  int start_i,end_i;
  if (hop_color == EMPTY) {
    return;
  }

  if (king) {
    start_i = start.i - 2;
    end_i = start.i + 2;
  }
  else {
    if (hop_color == WHITE) {
      start_i = end_i = start.i - 2;
    }
    else {
      start_i = end_i = start.i + 2;
    }
  }

  for (int i = start_i; i <= end_i; i+=4) {
    for (int j = start.j-2; j <= start.j+2; j+=4) {
      pos_end.i = i;
      pos_end.j = j;
      if (validPos(pos_end)) {
        pos_hop.i = (start.i+i)/2;
        pos_hop.j = (start.j+j)/2;

        if (!hopped[pos_hop.i][pos_hop.j] &&
            this->getPiece(pos_hop).type == hop_color &&
            (this->getPiece(pos_end).type == EMPTY || hopped[pos_end.i][pos_end.j])) {
           added_move = true;

           //recurse
           cur_move.push_back(pos_end);
           hopped[pos_hop.i][pos_hop.j] = true;
           hopDFS(pos_end, hop_color, king, cur_move, moves, hopped);
           hopped[pos_hop.i][pos_hop.j] = false;
           cur_move.pop_back();

        }
      }

    }
  }
  if (!added_move && cur_move.size() > 1) {
    moves.push_back(cur_move);
  }

}

void Checkers::printMoves(PieceType color) const {
  vector<move_t> moves = getMoves(color);
  for (auto m : moves) {
    for (auto p : m) {
      printf("(%d,%d) ",p.i,p.j);
    }
    printf("\n");
  }
}

void Checkers::printBoard() const {
  char buff[2];

  string board = "";
  board += "    0   1   2   3   4   5   6   7  \n";
  board += "  ---------------------------------\n";

  for (int i = 0; i < 8; ++i) {
    sprintf(buff,"%d",i);
    board += buff;
    board += " |";
    for (int j = 0; j < 8; ++j) {
      board += " ";
      board += pieceChar(this->getPiece(i,j));
      board += " |";
    }
    board += "\n";
    board += "  ---------------------------------\n";
  }
  printf("%s",board.c_str());
}

char pieceChar(piece p) {
  char ret_val;
  switch (p.type) {
    case WHITE: ret_val = p.king ? 'W' : 'w'; break;
    case BLACK: ret_val = p.king ? 'B' : 'b'; break;
    default:    ret_val = ' ';
  }
  return ret_val;
};

bool validPos(pos p) {
  return p.i>=0 && p.i<8 && p.j>=0 && p.j<8;
}
