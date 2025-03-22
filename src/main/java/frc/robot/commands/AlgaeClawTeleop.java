// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.OI;

public class AlgaeClawTeleop extends Command {
  AlgaeClaw algaeClaw;
  OI oi;

  public AlgaeClawTeleop(AlgaeClaw AlgaeClaw, OI Oi) {
    algaeClaw = AlgaeClaw;
    oi = Oi;
    addRequirements(algaeClaw);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(false/*oi.getIntakeAlgae()*/){
      algaeClaw.setAlgaeCollectorVel(.1);//TODO change velocities
    }
    else if(false/*oi.getScoreAlgae()*/){
      algaeClaw.setAlgaeCollectorVel(-.1);//TODO change velocities
    }
    else{
      algaeClaw.setAlgaeCollectorVel(0);
    }

    if(false/*oi.algaeToggle*/){
      if(algaeClaw.getIsEndeffectorRotatorUp()){
        algaeClaw.setAlgaeCollectorVel(1);
      }
      else{
        algaeClaw.setAlgaeCollectorVel(-1);
      }
    }
    else{
      algaeClaw.setAlgaeCollectorVel(0);
    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
