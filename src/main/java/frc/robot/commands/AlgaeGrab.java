// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeGrab extends Command {
  /** Creates a new AlgaeGrab. */
  CoralEndeffector coralEndeffector;
  public AlgaeGrab(CoralEndeffector coralEndeffector) {
    this.coralEndeffector = coralEndeffector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralEndeffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(coralEndeffector.getLoad() < 10){
      coralEndeffector.setVelocity(-30);
    }
    else{
      coralEndeffector.setVelocity(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEndeffector.setVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
