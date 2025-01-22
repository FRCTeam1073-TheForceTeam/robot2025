// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevator extends SubsystemBase {
  /** Creates a new CoralElevator. */
  private double position;
  private double velocity;
  private double load;

  public CoralElevator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getPosition(){// where motor is
    return position;
  }

  public void setZero(){
    position = 0.0;
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public boolean isCoralElevatorAtBottom(){
    if (position == 0.0){
      return true;
    }
    return false;
  }

  public void setPosition(double position){
    this.position = position;
  }

  public double getLoad(){
    return load;
  }

  public double getVelocity(){
    return velocity;
  }

}