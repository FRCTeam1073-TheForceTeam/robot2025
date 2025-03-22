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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClaw extends SubsystemBase {

  private double algaeCollectVel;
  private double algaeCollectPos;
  private double algaeCollectLoad;
  private double endeffectorRotateVel;
  private double endeffectorRotatePos;
  private double endeffectorRotateLoad;

  private boolean hasAlgae;
  private boolean algaeCollectBrakeMode;
  private boolean endeffectorRotateBrakeMode;

  private TalonFX algaeCollectMotor;
  private TalonFX endeffectorRotateMotor;
  private CANcoder encoder;
  private VelocityVoltage algaeCollectMotorVelocityVoltage;
  private PositionVoltage algaeCollectMotorPositionVoltage;
  private VelocityVoltage endeffectorRotatorMotorVelocityVoltage;
  private PositionVoltage endeffectorRotatorMotorPositionVoltage;

  private final double algaeCollectMinPos = 0; //TODO set min and max positions
  private final double algaeCollectMaxPos = 0;
  private final double endeffectorRotateMinPos = 0;
  private final double endeffectorRotateMaxPos = 0;

  private final double algaeCollectMotorKP = 0.15; //TODO change these values
  private final double algaeCollectMotorKI = 0;
  private final double algaeCollectMotorKD = 0;
  private final double algaeCollectMotorKV = 0.12;
  private final double endeffectorRotateMotorKP = 0.15;
  private final double endeffectorRotateMotorKI = 0;
  private final double endeffectorRotateMotorKD = 0;
  private final double endeffectorRotateMotorKV = 0.12;

  /** Creates a new AlgaeClaw. */
  public AlgaeClaw() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setAlgaeCollectorVel(double newVel){
    algaeCollectVel = newVel;
  }

  public void setEndffectorRotatorVel(double newVel){
    endeffectorRotateVel = newVel;
  }

  public void setAlgaeCollectorMotorBrakeMode(boolean mode){
    if(mode){
      algaeCollectMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      algaeCollectMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    algaeCollectBrakeMode = mode;
  }

  public void setEndeffectorRotateMotorBrakeMode(boolean mode){
    if(mode){
      endeffectorRotateMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      endeffectorRotateMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    endeffectorRotateBrakeMode = mode;
  }

  public double getAlgaeCollectorVelocity(){
    return algaeCollectVel;
  }

  public double getAlgaeCollectorPosition(){
    return algaeCollectPos;
  }

  public double getAlgaeCollectorLoad(){
    return algaeCollectLoad;
  }

  public double getEndeffectorRotatorVelocity(){
    return endeffectorRotateVel;
  }

  public double getEndeffectorRotatorPosition(){
    return endeffectorRotatePos;
  }

  public double getEndeffectorRotatorLoad(){
    return endeffectorRotateLoad;
  }

  public boolean getHasAlgae(){
    return hasAlgae;
  }

  public boolean getIsAlgaeCollectorAtZero(){
    return Math.abs(algaeCollectPos - .01) <= 0;
  }

  public boolean getIsEndeffectorRotatorAtZero(){
    return Math.abs(endeffectorRotatePos - .01) <= 0;
  }

  public boolean getEndeffectorRotatorBrakeMode(){
    return endeffectorRotateBrakeMode;
  }

  public boolean getAlgaeCollectorBrakeMode(){
    return algaeCollectBrakeMode;
  }

  public void configureHardware()
  {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeCollectMotor.getConfigurator().apply(motorConfig);
    endeffectorRotateMotor.getConfigurator().apply(motorConfig);

    var algaeCollectMotorClosedLoopConfig = new SlotConfigs();
    algaeCollectMotorClosedLoopConfig.withKP(algaeCollectMotorKP);
    algaeCollectMotorClosedLoopConfig.withKI(algaeCollectMotorKI);
    algaeCollectMotorClosedLoopConfig.withKD(algaeCollectMotorKD);
    algaeCollectMotorClosedLoopConfig.withKV(algaeCollectMotorKV);

    algaeCollectMotor.getConfigurator().apply(algaeCollectMotorClosedLoopConfig, 0.5);

    var endeffectorRotateMotorClosedLoopConfig = new SlotConfigs();
    endeffectorRotateMotorClosedLoopConfig.withKP(endeffectorRotateMotorKP);
    endeffectorRotateMotorClosedLoopConfig.withKI(endeffectorRotateMotorKI);
    endeffectorRotateMotorClosedLoopConfig.withKD(endeffectorRotateMotorKD);
    endeffectorRotateMotorClosedLoopConfig.withKV(endeffectorRotateMotorKV);

    endeffectorRotateMotor.getConfigurator().apply(endeffectorRotateMotorClosedLoopConfig, 0.5);

    algaeCollectMotor.setNeutralMode(NeutralModeValue.Coast);
    endeffectorRotateMotor.setNeutralMode(NeutralModeValue.Coast);

    CurrentLimitsConfigs motorCurrentLimitsConfigs = new CurrentLimitsConfigs(); //TODO check if these next 4 lines need to be copied
    motorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(15)
                            .withSupplyCurrentLowerTime(0.25);

    algaeCollectMotor.getConfigurator().apply(motorCurrentLimitsConfigs);
    endeffectorRotateMotor.getConfigurator().apply(motorCurrentLimitsConfigs);
   
    algaeCollectMotor.setPosition(0);
    endeffectorRotateMotor.setPosition(0);

    System.out.println("AlgaeCollector & EndeffectorRotator: Configured");
  }
}
