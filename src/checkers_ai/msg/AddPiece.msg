## Message to add/remove a piece from the checkers board

# If true, the board will be completely erased before the piece is added
bool clear

# If true, the board will be reset to the starting position and the rest of the message will be ignored
bool reset

# If true, the piece will be removed instead of added
bool remove

# Gives the position of the piece to be added
# If the position is invalid, no piece is added
int8 i
int8 j

# True for white, false for black
bool color

bool king
