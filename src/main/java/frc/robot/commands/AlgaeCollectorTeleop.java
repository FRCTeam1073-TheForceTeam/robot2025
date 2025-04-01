// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeCollector;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.OI;

public class AlgaeCollectorTeleop extends Command {
  AlgaeCollector algaeCollector;
  OI oi;

  private double velocity;
  private final double algaeVel = 30;

  public AlgaeCollectorTeleop(AlgaeCollector algaeCollector, OI OI) {
    this.algaeCollector = algaeCollector;
    oi = OI;
    addRequirements(algaeCollector);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    // //rotator logic from controller
    // if(oi.getOperatorAlgaeToggle() && clawUp) {
    //   algaeClaw.setRotatorPos(28.476);
    // }
    // else if(oi.getOperatorAlgaeToggle() && !clawUp) {
    //   algaeClaw.setRotatorPos(8.7);
    // }

    // if(oi.getOperatorLoadAlgae()) {
    //   algaeClaw.setCollectorVel(algaeVel);
    // }
    // else if(oi.getOperatorScoreAlgae()) {
    //   algaeClaw.setCollectorVel(-algaeVel);
    // }
    // else {
    //   algaeClaw.setCollectorVel(0);
    // }

    algaeCollector.setCollectorVel(0);

    SmartDashboard.putBoolean("AlgaeClaw/Algae Toggle Button", oi.getOperatorHighAlgae());
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
