// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TroughRaiseElevator extends Command 
{
  /** Creates a new TroughScoreCoral. */
  CoralElevator elevator;
  double velocity;
  double targetHeight = 12.8;
  // height 12.8
  public TroughRaiseElevator(CoralElevator coralElevator) 
  {
    elevator = coralElevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    velocity = (targetHeight - elevator.getPosition()) * 0.6;
    if (targetHeight > elevator.getPosition())
    {
      velocity = MathUtil.clamp(velocity, 3, 12);
    }
    else 
    {
      velocity = MathUtil.clamp(velocity, -12, -3);  
    }
    elevator.setVelocity(velocity);
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getPosition() - targetHeight) < (0.01 * targetHeight);
  }
}
