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
| LaserCAN Coral        |  22    |   Rio         |
| LaserCAN Reef         |  24    |   Rio         |
| CANdle                |  30    |   Rio         |
| PDH                   |  36    |   Rio         |

## Driver Controller

|  Button/Joystick | Function/Command               |
|------------------|--------------------------------|
|A/P3              |AlignToTag Center               |
|B/P1              |AlignToTag Source               |
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
|View Button       |Align To Source                 |
|Menu Button       |Local Align (temp)              |


## Operator Controller

|  Button/Joystick | Function/Command         |
|------------------|--------------------------|
|A                 |Disengage Climber         |
|B                 |Zero Climber              |
|X                 |Load Coral                |
|Y                 |Score Coral               |
|LeftJoystickY     |Coral Elevator            |
|RightJoystickY    |                          |
|LeftJoystickX     |                          |
|RightJoystickX    |                          |
|PressLeftJoystick |Stow Elevator             |
|PressRightJoystick|Cancel Load Coral         |
|DPadUp            |Elavtor Trough Level      |
|DPadDown          |Elevator Branch Level 3   |
|DPadLeft          |Elevaotr Branch Level 4   |
|DPadRight         |Elevator Branch Level 2   |
|LeftBumper        |Climber Up                |
|RightBumper       |Climber Down              |
|LeftTrigger       |Algae Remover             |
|RightTrigger      |Coral Intake & Shoot      |
|View Button       |Zero Elevator             |
|Menu Button       |Engage Climber            |

## Robot LED codes

| Light/Color                   | action                            |
|-------------------------------|-----------------------------------|
|Sides of funnel - light on     |Robot has coral                    |
|Sides of funnel - light off    |Robot does not have coral          |
|CANdle - green                 |Battery > 12 volts                 |
|CANdle - yellow                |Battery > 10 volts                 |
|CANdle - red                   |Battery <= 10 volts                |
|Front of elevator - green      |Robot endeffector sees reef        |
|Front of elevator - red        |Robot endeffector does not see reef|
|Side of elevator - blue        |Climber is disengaged              |
|Side of elevator - orange      |Climber is engaged                 |
|Side of elevator - purple      |Climber is zeroed                  |
|Side of elevator - grey        |Climber is between positions       |
|All funnel & elevator - blue   |Robot is disabled & blue alliance  |
|All funnel & elevator - red    |Robot is disabled & red alliance   |
|All funnel & elevator - white  |Robot is disabled & no  alliance   |

## Camera Names

FrontCenterCamera
FrontLeftCamera
FrontRightCamera
RearCamera