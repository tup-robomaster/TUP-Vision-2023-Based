# cmd msg for controlling gimbal
# id of task
uint8 tid

# constants for control type
uint8 ABSOLUTE_ANGLE = 1    # position control, set position by absolute angle
uint8 RELATIVE_ANGLE = 2    # position control, set position by relative angle
uint8 VELOCITY = 3          # velocity control, set velocity
uint8 FOLLOW_CHASSIS = 4    # follow chassis, only used by yaw_type.

# control type
uint8 yaw_type 1
uint8 pitch_type 1

# control dada
Gimbal position
Gimbal velocity
