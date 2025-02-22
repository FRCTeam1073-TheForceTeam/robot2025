// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroClimber extends Command {

  private double velocity;
  private Climber climber;
  private OI oi;

  /** Creates a new ZeroClimber. */
  public ZeroClimber(Climber Climber, OI Oi) {
    climber = Climber;
    oi = Oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climber.getEncoderPosition() > 0){
      velocity = -10;
    }
    else{
      velocity = 10;
    }

    climber.setCommandedVelocity(velocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return climber.getEncoderPosition() > -.01 && climber.getEncoderPosition() < .01;
  }
}
