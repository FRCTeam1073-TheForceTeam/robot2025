// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandStates extends SubsystemBase {
  /** Creates a new CommandStates. */
  private boolean isCollecting;
  private boolean isGlobalAligning;
  private boolean isLocalAligning;
  private boolean isLidarAligning;

  private boolean isGlobalAligned;
  private boolean isLocalAligned;
  private boolean isLidarAligned;


  public CommandStates() {
    isCollecting = false;

    isGlobalAligning = false;
    isLocalAligning = false;
    isLidarAligning = false;

    isGlobalAligned = false;
    isLocalAligned = false;
    isLidarAligned = false;
  }

  public void setIsGlobalAligning(boolean val) {
    isGlobalAligning = val;
  }

  public boolean getIsGlobalAligning() {
    return isGlobalAligning;
  }

  public void setIsGlobalAligned(boolean val) {
    isGlobalAligned = val;
  }

  public boolean getIsGlobalAligned() {
    return isGlobalAligned;
  }

  public void setIsLocalAligning(boolean val) {
    isLocalAligning = val;
  }

  public boolean getIsLocalAligning() {
    return isLocalAligning;
  }

  public boolean getIsLidarAligned() {
    return isLidarAligned;
  }

  public void setIsLocalAligned(boolean val) {
    isLocalAligned = val;
  }

  public void setIsLidarAligning(boolean val) {
    isLidarAligning = val;
  }

  public boolean getIsLidarAligning() {
    return isLidarAligning;
  }

  public boolean getIsLocalAligned() {
    return isLocalAligned;
  }

  public void setIsLidarAligned(boolean val) {
    isLidarAligned = val;
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

    //if aligned then you are done aligning
    if(isGlobalAligned) {
      isGlobalAligning = false;
    }
    if(isLocalAligned) {
      isLocalAligning = false;
    }
    if(isLidarAligned) {
      isLidarAligning = false;
    }

    SmartDashboard.putBoolean("States/isCollecting", isCollecting);
    SmartDashboard.putBoolean("States/isGlobalAligning", isGlobalAligning);
    SmartDashboard.putBoolean("States/isGlobalAligned", isGlobalAligned);
    SmartDashboard.putBoolean("States/isLocalAligning", isLocalAligning);
    SmartDashboard.putBoolean("States/isLocalAligned", isLocalAligned);
    SmartDashboard.putBoolean("States/isLidarAligning", isLidarAligning);
    SmartDashboard.putBoolean("States/isLidarAligned", isLidarAligned);
  }
}
