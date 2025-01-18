// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberLift extends SubsystemBase {

  private double load;
  private double velocity;
  private double position;
  private boolean brakeMode;
  


  /* Creates a new ClimberLift. */
  public ClimberLift() {}

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

  public void setZero(){
    position = 0.0;
  }

  public void setBrakeMode(Boolean mode){
    brakeMode = mode;
  }


  public boolean getBrakeMode(){
    return brakeMode;
  }

  public double getPosition(){
    return position;
  }
}
