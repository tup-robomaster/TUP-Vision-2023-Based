# msg for cmd in referee system
uint8 START_PREPARATION = 0    # system cmd: transit to PREPARATION stage
uint8 START_SELF_CHECKING = 1  # system cmd: transit to SELF_CHECKING stage
uint8 START_GAME = 2           # system cmd: transit to RUNNING stage
uint8 STOP_GAME = 3            # system cmd: transit to GAME_OVER stage
uint8 KILL_ROBOT = 4           # control robot cmd: kill robot with robot_name
uint8 REVIVE_ROBOT = 5         # control robot cmd: revive robot with robot_name

uint8 cmd 0
string robot_name

# deprecate constant definition (don't use them)
uint8 PREPARATION = 0
uint8 SELF_CHECKING = 1
