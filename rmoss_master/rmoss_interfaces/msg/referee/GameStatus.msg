# msg for referee system
# game status
uint8 PREPARATION = 0    # preparation stage (free control for robot, disable referee system)
uint8 SELF_CHECKING = 1  # referee system self-checking stage (unable to control robot, and 
                         # initialize all referee system data).
uint8 RUNNING = 2        # running stage (free control for robot, enable referee system)
uint8 GAME_OVER = 3      # game over stage (unable to control robot, disable referee system)

uint8 status 0

# game time
int32 total_time 0         # seconds
int32 current_time 0       # seconds

# game result
uint8 DRAW = 0
uint8 RED_WIN = 1
uint8 BLUE_WIN = 2

uint8 result
