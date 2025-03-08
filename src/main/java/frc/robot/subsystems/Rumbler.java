// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rumbler extends SubsystemBase {

  Drivetrain drivetrain;
  SwerveModule fL;
  SwerveModule fR;
  SwerveModule bL;
  SwerveModule bR;

  double frontLeftTorque;
  double frontRightTorque;
  double backLeftTorque;
  double backRightTorque;
  double avgTorque;

  double torqueGate = 50; 

  /** Creates a new Rumbler. */
  public Rumbler(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.fL =  drivetrain.getModules()[0];
    this.fR = drivetrain.getModules()[1];
    this.bL = drivetrain.getModules()[2];
    this.bR = drivetrain.getModules()[3];
  }

  public static void rumble() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 1);
  }

  public static void stopRumble() {
    OI.driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void periodic() {
    //get the absolute value of the load from the drive motors
    frontLeftTorque = fL.getLoad();
    frontRightTorque = fR.getLoad();
    backLeftTorque = bL.getLoad();
    backRightTorque = bL.getLoad();

    avgTorque = (frontLeftTorque + frontRightTorque + backLeftTorque + backRightTorque) / 4;

    //if above the torque gate rumble the contorller
    if(avgTorque >= torqueGate) {
      rumble();
    }
    else if(avgTorque < torqueGate) {
      stopRumble();
    }

    SmartDashboard.putNumber("Avg Torque", avgTorque);
  }
}
