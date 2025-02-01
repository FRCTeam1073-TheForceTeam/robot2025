// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ClimberLift;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberLiftTeleop extends Command {

  ClimberLift lift;
  OI oi;
  private double velocity;

  /** Creates a new ClimberLiftTeleop. */
  public ClimberLiftTeleop(ClimberLift lift, OI Oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.lift = lift;
    this.oi = Oi;
    addRequirements(lift);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lift.setBrakeMode(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = oi.getOperatorRightY();
    lift.setVelocity(velocity);
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
