// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleControl;
import frc.robot.subsystems.CoralEndeffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CANdleObserver extends Command {
  
  CANdleControl candleControl;
  CoralEndeffector endeffector;

  public CANdleObserver(CANdleControl candleControl, CoralEndeffector endeffector) {
    this.candleControl = candleControl;
    this.endeffector = endeffector;
    addRequirements(candleControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (endeffector.getHasReef()){
      candleControl.setRGB(0, 255, 0, 0, 37);
    }

    else{
      candleControl.setRGB(255, 0, 0, 0, 37);
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
