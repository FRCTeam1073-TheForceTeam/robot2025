// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.CommandStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAutoGrabCommand extends Command {
  /** Creates a new AlgaeAutoGrabCommand. */
  AlgaeCollector algaeCollector;
  CommandStates state;

  public AlgaeAutoGrabCommand(AlgaeCollector algaeCollector, CommandStates states) {
    this.algaeCollector = algaeCollector;
    this.state = states;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeCollector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state.setIsCollecting(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(algaeCollector.getCollectorLoad() < 10){
      algaeCollector.setCollectorVel(10);
    }
    else{
      algaeCollector.setCollectorVel(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeCollector.setCollectorVel(0);
    state.setIsCollecting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
