# robot2025

## Direction Conventions
TBD

Swerve module #0 is front left, #1 is front right, #2 is back left, and #3 is back right.

##Module CAN IDs
| Device                | CAN ID |      BUS      |
| --------------------- | ------ | ------------- |
| Swerve #0 Encoder     |   1    |   CANivore    |
| Swerve #1 Encoder     |   2    |   CANivore    |
| Swerve #2 Encoder     |   3    |   CANivore    |
| Swerve #3 Encoder     |   4    |   CANivore    |
| Swerve #0 Drive Motor |   5    |   CANivore    |
| Swerve #0 Steer Motor |   6    |   CANivore    |
| Swerve #1 Drive Motor |   7    |   CANivore    |
| Swerve #1 Steer Motor |   8    |   CANivore    |
| Swerve #2 Drive Motor |   9    |   CANivore    |
| Swerve #2 Steer Motor |  10    |   CANivore    |
| Swerve #3 Drive Motor |  11    |   CANivore    |
| Swerve #3 Steer Motor |  12    |   CANivore    |
| Pigeon 2              |  13    |   CANivore    |
| Climber Motor         |  15    |   Rio         |
| Climber Encoder       |  16    |   Rio         |
| Claw Motor left       |  17    |   Rio         |
| Claw Motor right      |  18    |   Rio         |
| Elevator Motor left   |  19    |   Rio         |
| Elevator Motor right  |  20    |   Rio         |
| Endeffector Motor     |  21    |   Rio         |
| LaserCAN              |  22    |   Rio         |
| CANdle                |  30    |   Rio         |
| PDH                   |  36    |   Rio         |
| Collector Motor       |   ?    |   CANivore    |
| Lift Motor            |   ?    |   CANivore    |
| Extend Motor          |   ?    |   CANivore    |
| Top Shooter Motor     |   ?    |   CANivore    |
| Bottom Shooter Motor  |   ?    |   CANivore    |
| Leader Feeder Motor   |   ?    |   CANivore    |
| Follower Feeder Motor |   ?    |   CANivore    |
| Pivot Motor           |   ?    |   CANivore    |

## Driver Controller

|  Button/Joystick | Function/Command               |
|------------------|--------------------------------|
|A/P3              |AlignToTag Center               |
|B/P1              |Align To Source                 |
|X/P4              |AlignToTag Left                 |
|Y/P2              |AlignToTag Right                |
|LeftJoystick      |Translation                     |
|RightJoystick     |Rotation                        |
|PressLeftJoystick |                                |
|PressRightJoystick|                                |
|DPadUp            |Creep Forward                   |
|DPadDown          |Creep Backward                  |
|DPadLeft          |Creep Left                      |
|DPadRight         |Creep Right                     |
|LeftBumper        |Parking brake                   |
|RightBumper       |Field-centric/robot-centric     |
|LeftTrigger       |Increase speed                  |
|RightTrigger      |Increase speed                  |
|View Button       |Correction Align                |
|Menu Button       |Reset Odometry                  |


## Operator Controller

|  Button/Joystick | Function/Command         |
|------------------|--------------------------|
|A                 |Disenage Climber          |
|B                 |Engage Climber            |
|X                 |Load Coral                |
|Y                 |Score Coral               |
|LeftJoystickY     |Coral Elevator            |
|RightJoystickY    |                          |
|LeftJoystickX     |                          |
|RightJoystickX    |                          |
|PressLeftJoystick |Zero Elevator             |
|PressRightJoystick|Cancel Load Coral         |
|DPadUp            |Elavtor Trough Level      |
|DPadDown          |Elevator Branch Level 3   |
|DPadLeft          |Elevaotr Branch Level 4   |
|DPadRight         |Elevator Branch Level 2   |
|LeftBumper        |Climber Up                |
|RightBumper       |Climber Down              |
|LeftTrigger       |Algae Remover             |
|RightTrigger      |Coral Intake & Shoot      |
|View Button       |Coral Endeffector Teleop  |
|Menu Button       |Zero Climber              |
