// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimberClaw;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

/** Add your docs here. */
public class AutoCenterLeftStart 
{
   public static Command create(int level, boolean isRed, Drivetrain drivetrain, Localizer localizer, FieldMap map, ClimberClaw claw, ClimberLift lift, double delay)
    {
        switch (level)
        {
            case 0: 
               return GenericL0.create(isRed, drivetrain, localizer, claw, lift);
            case 1: 
               return CenterL1.create(isRed, drivetrain, map, localizer);
            case 2:
               return CenterLeftL2.create(isRed, drivetrain, map, localizer, delay);
            case 3:
               return CenterLeftL3.create(isRed, drivetrain, delay);
            case 4:
               return CenterLeftL4.create(isRed, drivetrain, delay);
            default:
               return new WaitCommand(0);
        }
    }
}