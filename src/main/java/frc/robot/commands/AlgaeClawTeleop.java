// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OldOI;

public class AlgaeClawTeleop extends Command {
  AlgaeClaw algaeClaw;
  OldOI oi;
  private double velocity;

  public AlgaeClawTeleop(AlgaeClaw AlgaeClaw, OldOI OI) {
    algaeClaw = AlgaeClaw;
    oi = OI;
    addRequirements(algaeClaw);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    velocity = oi.getOperatorLeftX() * 12.0;//TODO change controls
    algaeClaw.setRotatorVel(velocity);

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
