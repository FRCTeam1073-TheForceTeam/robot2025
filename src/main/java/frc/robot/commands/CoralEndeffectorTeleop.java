// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.CoralEndeffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralEndeffectorTeleop extends Command {

  CoralEndeffector endeffector;
  OI oi;
  private double velocity = 0.5;

  /** Creates a new CoralEndeffectorTeleop. */
  public CoralEndeffectorTeleop(CoralEndeffector Endeffector, OI Oi){
    // Use addRequirements() here to declare subsystem dependencies.
    endeffector = Endeffector;
    oi = Oi;
    addRequirements(endeffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorLeftTrigger()){
      endeffector.setVelocity(velocity);
    }
    else{
      endeffector.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
