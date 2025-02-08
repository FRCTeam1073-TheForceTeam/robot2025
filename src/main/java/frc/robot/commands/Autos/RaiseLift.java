// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberClaw;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseLift extends Command {
  ClimberLift lift;
  OI oi;
  // TODO: make this 67
  double targetPosition = 66.5;
  /** Creates a new RaiseLift. */
  public RaiseLift(ClimberLift lift, OI oi) {
    this.lift = lift;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lift);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lift.getPosition() < targetPosition){
      double velocity = (targetPosition - lift.getPosition()) * 0.6;
      velocity = MathUtil.clamp(velocity, 3, 12);
      lift.setVelocity(velocity);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lift.setVelocity(0);
    lift.setBrakeMode(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lift.getPosition() >= targetPosition;
  }
}
