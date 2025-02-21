// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;


public class EngageClimber extends Command 
{
  Climber climber;

  public EngageClimber(Climber climber) 
  {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() 
  {

  }

  @Override
  public void execute() 
  {
    climber.setCommandedVelocity(0);
  }

  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
