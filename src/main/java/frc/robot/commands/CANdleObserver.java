// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleControl;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CANdleObserver extends Command {
  
  CANdleControl candleControl;
  CoralEndeffector endeffector;
  Climber climber;
  OI oi;


  public CANdleObserver(CANdleControl CandleControl, CoralEndeffector Endeffector, Climber Climber, OI Oi) {
    candleControl = CandleControl;
    endeffector = Endeffector;
    climber = Climber;
    oi = Oi;
    addRequirements(candleControl);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    candleControl.clearAnim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (endeffector.getHasReef()){
      candleControl.setRGB(0, 255, 0, 8, 16);//elevator forward - green
    }
    else{
      candleControl.setRGB(255, 0, 0, 8, 16);//elevator forward - red
    }

    if (endeffector.getHasCoral()){
      candleControl.setRGB(255, 255, 255, 37, 29);//sides of funnel - light on
    }
    else{
      candleControl.setRGB(0, 0, 0, 37, 29);//sides of funnel - light off
    }

    if (RobotController.getBatteryVoltage() > 12){
      candleControl.setRGB(0, 255, 0, 0, 8);//CANdle - green
    }
    else if(RobotController.getBatteryVoltage() > 10){
      candleControl.setRGB(128, 128, 0, 0, 8);//CANdle - yellow
    }
    else{
      candleControl.setRGB(255, 0, 0, 0, 8);//CANdle - red
    }

    if (climber.getIsDisengaged()){
      candleControl.setRGB(0, 0, 255, 24, 13);//elevator side - blue
    }
    else if (climber.getIsEngaged()){
      candleControl.setRGB(245, 146, 0, 24, 13);//elevator side - orange
    }
    else if (climber.getIsAtZero()){
      candleControl.setRGB(255, 0, 255, 24, 13);//elevator side - purple
    }
    else{
      candleControl.setRGB(128, 128, 128, 24, 13);//elevator side - grey
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