// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/** Add your docs here. */
public class AutoCenterRightStart 
{
    public static Command create(int level, boolean isRed, Drivetrain drivetrain, Localizer localizer, FieldMap map, 
                                     Climber climber, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar)
    {
        switch (level)
        {
            case 0: 
               return Leave.create(isRed, drivetrain, localizer, climber);
            case 1: 
               return CenterRightScoreL1.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar);
            case 2:
               return CenterRightScoreL2.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar);
            case 3:
               return CenterRightScoreL3.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar);
            case 4:
               return CenterRightScoreL4.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar);
            case 5:
               return CenterRightScoreL4.create(isRed, drivetrain, map, localizer, endEffector, elevator, lidar);
            default:
               return new WaitCommand(0);
        }
    }
}