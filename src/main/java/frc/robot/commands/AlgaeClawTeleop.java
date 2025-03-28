// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.OI;

public class AlgaeClawTeleop extends Command {
  AlgaeClaw algaeClaw;
  OI oi;

  private double velocity;
  private boolean clawUp;
  private final double algaeVel = 50;

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
    clawUp = algaeClaw.getIsUp();

    //rotator logic from controller
    if(oi.getOperatorAlgaeToggle() && clawUp) {
      algaeClaw.setRotatorPos(28.476);
    }
    else if(oi.getOperatorAlgaeToggle() && !clawUp) {
      algaeClaw.setRotatorPos(8.7);
    }
    else {
      velocity = oi.getOperatorLeftX() * 7.0;
      algaeClaw.setRotatorVel(velocity);
    }

    //collector logic from controller
    if(oi.getOperatorLoadAlgae()){
      algaeClaw.setCollectorVel(30);
    } else if(oi.getOperatorScoreAlgae()){
      algaeClaw.setCollectorVel(-30);
    }
    else{
      algaeClaw.setCollectorVel(0);
    }

    if(oi.getOperatorLoadAlgae()) {
      algaeClaw.setCollectorVel(algaeVel);
    }
    else if(oi.getOperatorScoreAlgae()) {
      algaeClaw.setCollectorVel(-algaeVel);
    }
    else {
      algaeClaw.setCollectorVel(0);
    }

    SmartDashboard.putBoolean("AlgaeClaw/Algae Toggle Button", oi.getOperatorAlgaeToggle());
    SmartDashboard.putBoolean("AlgaeClaw/Algae Score Button", oi.getOperatorScoreAlgae());
    SmartDashboard.putBoolean("AlgaeClaw/Algae Load Button", oi.getOperatorLoadAlgae());
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
