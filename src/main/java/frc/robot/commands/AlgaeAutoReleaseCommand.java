// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.Timestamp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClaw;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAutoReleaseCommand extends Command {
  /** Creates a new AlgaeAutoReleaseCommand. */
  AlgaeClaw algaeClaw;
  double timeAtInit;
  public AlgaeAutoReleaseCommand(AlgaeClaw algaeClaw) {
    this.algaeClaw = algaeClaw;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeAtInit = Timer.getFPGATimestamp();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeClaw.setCollectorVel(-40);
    //algaeClaw.setRotatorVel(15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeClaw.setCollectorVel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timeAtInit > 5.0) {
      return true;
    }
    return false;
  }
}
