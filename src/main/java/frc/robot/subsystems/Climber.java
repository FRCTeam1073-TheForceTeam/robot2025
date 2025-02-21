// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase 
{
  private TalonFX motor;
  private CANcoder encoder;
  private VelocityVoltage motorVelocityVoltage;
  private PositionVoltage motorPositionVoltage;
  public Debouncer climberDebouncer = new Debouncer(0.05);

  private final double motorKP = 0.1;
  private final double motorKI = 0;
  private final double motorKD = 0;
  private final double motorKV = 0.12;

  private double velocity = 0;
  private double position = 0;
  private double load = 0;
  private double commandedVelocity = 0;
  private boolean brakeMode = true;


  /** Creates a new Climber. */
  public Climber() 
  {
    motor = new TalonFX(15);
    encoder = new CANcoder(16);

    motorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    motorPositionVoltage = new PositionVoltage(0).withSlot(0);
  }

  @Override
  public void periodic() 
  {
    velocity = motor.getVelocity().refresh().getValueAsDouble(); 
    position = motor.getPosition().refresh().getValueAsDouble();
    load = motor.getTorqueCurrent().getValueAsDouble();

    motor.setControl(motorVelocityVoltage.withVelocity(commandedVelocity));
  }

  public void setCommandedVelocity(double velocity)
  {
    commandedVelocity = velocity;
  }

  public double getVelocity()
  {
    return velocity;
  }

  public double getMotorPosition()
  {
    return position;
  }

  public double getLoad()
  {
    return load;
  }

  public double getEncoderPosition()
  {
    return encoder.getAbsolutePosition().refresh().getValueAsDouble();
  }

  public void setZero()
  {
    position = 0;
    motor.setPosition(0);
  }

  public void setBrakeMode(boolean mode)
  {
    if (mode)
    {
      motor.setNeutralMode(NeutralModeValue.Brake);
    }
    else
    {
      motor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakeMode = mode;
  }

  public void configureHardware()
  {
    var motorConfig = new TalonFXConfiguration();
    motor.getConfigurator().apply(motorConfig);

    var motorClosedLoopConfig = new SlotConfigs();
    motorClosedLoopConfig.withKP(motorKP);
    motorClosedLoopConfig.withKI(motorKI);
    motorClosedLoopConfig.withKD(motorKD);
    motorClosedLoopConfig.withKV(motorKV);

    motor.setNeutralMode(NeutralModeValue.Brake);

    CurrentLimitsConfigs motorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    motorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(15)
                            .withSupplyCurrentLowerTime(0.25);

    motor.getConfigurator().apply(motorCurrentLimitsConfigs);

    motor.setPosition(0);

    System.out.println("Climber: Configured");
  }
}
