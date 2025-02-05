// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.ClimberClaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberClawTeleop extends Command {

  ClimberClaw claw;
  OI oi;
  private double velocity;

  public ClimberClawTeleop(ClimberClaw Claw, OI Oi) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.claw = Claw;
    this.oi = Oi;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    claw.setBrakeMode(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(oi.getOperatorLeftBumper()){
      velocity = 2;
    }
    else if(oi.getOperatorRightBumper()){
      velocity = -2;
    }
    else{
      velocity = 0;
    }
    claw.setVelocity(velocity);
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
