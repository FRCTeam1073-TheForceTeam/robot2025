// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.security.Timestamp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAutoReleaseCommand extends Command {
  /** Creates a new AlgaeAutoReleaseCommand. */
  AlgaeCollector algaeCollector;
  double timeAtInit;
  public AlgaeAutoReleaseCommand(AlgaeCollector algaeCollector) {
    this.algaeCollector = algaeCollector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeAtInit = Timer.getFPGATimestamp();    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeCollector.setCollectorVel(-85);
    //algaeClaw.setRotatorVel(15);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeCollector.setCollectorVel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - timeAtInit > 2.0) {
      return true;
    }
    return false;
  }
}
