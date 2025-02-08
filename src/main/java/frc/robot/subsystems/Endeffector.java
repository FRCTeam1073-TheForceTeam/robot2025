// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Endeffector extends SubsystemBase {

  private final String kCANbus = "rio";

  private double velocity = 0.0;

  private TalonFX effectorMotor;

  private final double KP = 0.1; // TODO: tune values
  private final double KI = 0.0;
  private final double KD = 0.0;
  private final double KV = 0.0;

  //time of flight stuff
  private final DigitalInput effectorTof;
  private final DutyCycle effectorTofDutyCycleInput;
  private final double effectorTofScaleFactor = 0; //TODO: FIND SCALE FACTOR!!!!!!!
  private double effectorTofFreq;
  private double effectorTofRange;
  private double effectorTofDutyCycle;

  private VelocityVoltage effectorVelocityVoltage = new VelocityVoltage(0);

  /** Creates a new Endeffector. */
  public Endeffector() {

    effectorMotor = new TalonFX(21, kCANbus);
    //TODO: need motorFault and motorLimiter?

    effectorTof = new DigitalInput(1); // TODO: check to see if this is the right channel!
    effectorTofDutyCycleInput = new DutyCycle(effectorTof);
    effectorTofFreq = 0;
    effectorTofRange = 0;

    // configureHardware(); TODO: pls uncomment this later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    effectorTofFreq = effectorTofDutyCycleInput.getFrequency();
    effectorTofDutyCycle = effectorTofDutyCycleInput.getOutput();
    effectorTofRange = effectorTofDutyCycleInput.getOutput();
    effectorTofRange = (effectorTofScaleFactor * (effectorTofDutyCycle / effectorTofFreq - 0.001)) / 1000; // TODO: check to make sure numbers are right

  //TODO: configure hardware and finish getting Tof info

      SmartDashboard.putNumber("EndEffector/Velocity", velocity);

  }
}
