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

  public AlgaeClawTeleop(AlgaeClaw AlgaeClaw, OI OI) {
    algaeClaw = AlgaeClaw;
    oi = OI;
    addRequirements(algaeClaw);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(oi.getOperatorLoadAlgae()){
      algaeClaw.setCollectorVel(0.5);//TODO change velocities
    }
    else if(oi.getOperatorScoreAlgae()){
      algaeClaw.setCollectorVel(-0.5);
    }
    else{
      algaeClaw.setCollectorVel(0);
    }

    if(oi.getOperatorAlgaeToggle()){
        algaeClaw.toggleIsUp();
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
