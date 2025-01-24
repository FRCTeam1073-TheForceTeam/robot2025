// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class CoralEndeffector extends SubsystemBase {

    private double velocity;
    private double position;
    private double load;

    public CoralEndeffector(){

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public void setPosition(double position){
        this.position = position;
    }
    public double getPosition(){
        return position;
    }
    public void setVelocity(double velocity){
        this.velocity = velocity;
    }
    public double getVelocity(){
            return velocity;
    }
    public double getLoad(){
        return load;
    }
    public boolean getHasCoral(){
    //Pull from sensor
        return false;
    }

}