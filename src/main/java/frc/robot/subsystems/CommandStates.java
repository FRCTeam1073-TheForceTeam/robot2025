// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandStates extends SubsystemBase {
  /** Creates a new CommandStates. */
  private boolean isCollecting;
  private boolean isLidarAligning;

  public CommandStates() {
    isCollecting = false;
    isLidarAligning = false;
  }

  public void setIsLidarAligning(boolean val) {
    isLidarAligning = val;
  }

  public boolean getIsLidarAligning() {
    return isLidarAligning;
  }

  public void setIsCollecting(boolean val) {
    isCollecting = val;
  }

  public boolean getIsCollecting() {
    return isCollecting;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
