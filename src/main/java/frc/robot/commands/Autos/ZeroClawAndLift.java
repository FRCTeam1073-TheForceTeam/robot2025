// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ZeroClaw;
import frc.robot.commands.ZeroLift;
import frc.robot.subsystems.ClimberClaw;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.OI;

/** Add your docs here. */
public class ZeroClawAndLift {
    
    public static Command create(ClimberClaw claw, ClimberLift lift){
        return new ParallelCommandGroup(
            new ZeroClaw(claw),
            new ZeroLift(lift)
            );
    }
}
