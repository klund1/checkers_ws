# CheckersBot
## A Checkers Playing Robot

This is a ROS workspace containing all of the code needed to run a completely self-sufficient checkers playing robot.
The robot watches the checkers board, detects a move, plans its next move, and then moves the corresponding piece.

### High Level State Machine: `checkers_main`
This ROS package contains the implementation of the state machine that controls the system.

The possible states are:
 - Start (`START`)
 - Wait for human move (`HUMAN_MOVE`)
 - Compute move (`AI_COMPUTE`)
 - Move arm (`MOVE_ARM`)

The inputs the trigger state transitions are:
 - Reset (`RESET`, from button/user input)
 - Start game (`START`, from button/user input)
 - Got human move (`VIS:FINISHED`, from `checkers_vis`)
 - Computed next move (`AI:FINISHED`, from `checkers_ai`)
 - Finished moving piece (`ARM:FINISHED`, from `checkers_arm`)

Subscribes to:
 - `/events` (`std_msgs/String`)

Publishes to:
 - `/state` (`std_msgs/String`)

### CV System: `checkers_vis`

Subscribes to:
 - `/state` (`std_msgs/String`)

Publishes to:
 - `/events` (`std_msgs/String`)
 - `/add_piece` (`checkers_ai/addPiece`)

### AI: `checkers_ai`

Subscribes to:
 - `/state` (`std_msgs/String`)
 - `/add_piece` (`checkers_ai/addPiece`)

Publishes to:
 - `/events` (`std_msgs/String`)
 - `/move_piece` (`std_msgs/String`)

### Arm control: `checkers_arm`

Subscribes to:
 - `/state` (`std_msgs/String`)
 - `/move_piece` (`std_msgs/String`)

Publishes to:
 - `/events`

