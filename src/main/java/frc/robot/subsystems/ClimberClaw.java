// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberClaw extends SubsystemBase {
  /** Creates a new ClimberClaw. */
  private final double gearRatio = 8;
  private final double sprocketDiameter = 0.054864;
  private final double leftMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double rightMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;

  private double velocity;
  private double load;
  private boolean brakeMode;
  private boolean cageDetected;


  public ClimberClaw() {
    velocity = 0;
    brakeMode = false;
    cageDetected = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getLoad(){
    return load;
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public double getVelocity(){
    return velocity;
  }

  public void setZero(double zero){

  }
  
  public void setBrakeMode(Boolean mode){
    brakeMode = mode;
  }


  public boolean getBrakeMode(){
    return brakeMode;
  }

  public boolean getCageDetected(){
    return cageDetected;
  }
}
