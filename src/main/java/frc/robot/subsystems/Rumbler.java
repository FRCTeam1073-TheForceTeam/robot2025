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
  PowerDistribution fL;
  PowerDistribution fR;
  PowerDistribution bL;
  PowerDistribution bR;

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
    this.fL = new PowerDistribution(0, ModuleType.kCTRE);
    this.fR = new PowerDistribution(1, ModuleType.kCTRE);
    this.bL = new PowerDistribution(2, ModuleType.kCTRE);
    this.bR = new PowerDistribution(3, ModuleType.kCTRE);
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
    frontLeftVolt = fL.getVoltage();
    frontRightVolt = fR.getVoltage();
    backLeftVolt = bL.getVoltage();
    backRightVolt = bL.getVoltage();

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
