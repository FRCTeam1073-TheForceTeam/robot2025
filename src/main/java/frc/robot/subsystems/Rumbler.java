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

  double frontLeftVolt;
  double frontRightVolt;
  double backLeftVolt;
  double backRightVolt;
  double avgVolt;

  double voltGate = 0; //TODO: find out this number
  boolean lastRumbleOn = false;

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
    // This method will be called once per scheduler run
    frontLeftVolt = fL.getLoad();
    frontRightVolt = fR.getLoad();
    backLeftVolt = bL.getLoad();
    backRightVolt = bL.getLoad();

    avgVolt = (frontLeftVolt + frontRightVolt + backLeftVolt + backRightVolt) / 4;

    if(avgVolt >= voltGate && !lastRumbleOn) {
      rumble();
    }
    else if(avgVolt < voltGate && lastRumbleOn) {
      stopRumble();
    }

    SmartDashboard.putNumber("FL Velocity", frontLeftVolt);
    SmartDashboard.putNumber("FR Velocity", frontRightVolt);
    SmartDashboard.putNumber("BL Velocity", backLeftVolt);
    SmartDashboard.putNumber("BR Velocity", backRightVolt);
  }
}
