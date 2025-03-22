// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClaw extends SubsystemBase {

  private double algaeCollectVel;
  private double commandedAlgaeCollectVel;
  private double algaeCollectLoad;
  private double endeffectorRotateVel;
  private double commandedEndeffectorRotateVel;
  private double endeffectorRotatePos;
  private double endeffectorRotateLoad;

  private boolean hasAlgae;
  private boolean algaeCollectBrakeMode = true;
  private boolean endeffectorRotateBrakeMode = true;
  private boolean endeffectorIsUp = true;

  private TalonFX algaeCollectMotor;
  private TalonFX endeffectorRotateMotor;
  private VelocityVoltage algaeCollectMotorVelocityVoltage;
  private VelocityVoltage endeffectorRotateMotorVelocityVoltage;
  private MotionMagicVoltage endeffectorRotateMotorPositionVoltage;

  private final double endeffectorRotateMinPos = 0;//TODO set min and max positions
  private final double endeffectorRotateMaxPos = 5;

  private final double algaeCollectMotorKP = 0.15; //TODO change these values
  private final double algaeCollectMotorKI = 0;
  private final double algaeCollectMotorKD = 0;
  private final double algaeCollectMotorKV = 0.12;
  private final double endeffectorRotateMotorKP = 0.15;
  private final double endeffectorRotateMotorKI = 0;
  private final double endeffectorRotateMotorKD = 0;
  private final double endeffectorRotateMotorKV = 0.12;

  public AlgaeClaw() {
    algaeCollectMotor = new TalonFX(0);//TODO find device IDs
    endeffectorRotateMotor = new TalonFX(0);

    algaeCollectMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    endeffectorRotateMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    endeffectorRotateMotorPositionVoltage = new MotionMagicVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() {
    algaeCollectVel = algaeCollectMotor.getVelocity().getValueAsDouble(); 
    algaeCollectLoad = algaeCollectMotor.getTorqueCurrent().getValueAsDouble();
    endeffectorRotateVel = endeffectorRotateMotor.getVelocity().getValueAsDouble(); 
    endeffectorRotatePos = endeffectorRotateMotor.getPosition().getValueAsDouble();
    endeffectorRotateLoad = endeffectorRotateMotor.getTorqueCurrent().getValueAsDouble();

    if (endeffectorRotatePos >= endeffectorRotateMaxPos){
      commandedEndeffectorRotateVel = Math.min(commandedEndeffectorRotateVel, 0);
    }
    if (endeffectorRotatePos <= endeffectorRotateMinPos){
      commandedEndeffectorRotateVel = Math.max(commandedEndeffectorRotateVel, 0);
    }
    if(endeffectorRotatePos <= 0.5){
      endeffectorIsUp = true;
    }

    algaeCollectMotor.setControl(algaeCollectMotorVelocityVoltage.withVelocity(commandedAlgaeCollectVel));
    endeffectorRotateMotor.setControl(endeffectorRotateMotorVelocityVoltage.withVelocity(commandedEndeffectorRotateVel));

    SmartDashboard.putNumber("[AlgaeClaw] algae collect velocity", algaeCollectVel);
    SmartDashboard.putNumber("[AlgaeClaw] algae collect commanded velocity", commandedAlgaeCollectVel);
    SmartDashboard.putNumber("[AlgaeClaw] endeffector rotate velocity", endeffectorRotateVel);
    SmartDashboard.putNumber("[AlgaeClaw] endeffector rotate position", endeffectorRotatePos);
    SmartDashboard.putNumber("[AlgaeClaw] endeffector rotate commanded velocity", commandedEndeffectorRotateVel);
    SmartDashboard.putBoolean("[AlgaeClaw] has algae", hasAlgae);
  }

  public void setAlgaeCollectorVel(double newVel){
    commandedAlgaeCollectVel = newVel;
  }

  public void setEndeffectorRotatorVel(double newVel){
    commandedEndeffectorRotateVel = newVel;
  }

  public void zeroEndeffectorRotator(){
    if(!this.getIsEndeffectorRotatorAtZero()){
      commandedEndeffectorRotateVel = -1;
    }
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

  public boolean getIsEndeffectorRotatorAtZero(){
    return endeffectorRotatePos <= .01;
  }

  public boolean getIsEndeffectorRotatorUp(){
    return endeffectorIsUp;
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

    var algaeCollectmmConfigs = motorConfig.MotionMagic;
    var endeffectorRotatemmConfigs = motorConfig.MotionMagic;
    algaeCollectmmConfigs.MotionMagicCruiseVelocity = 100;
    algaeCollectmmConfigs.MotionMagicAcceleration = 100;
    algaeCollectmmConfigs.MotionMagicJerk = 100;
    endeffectorRotatemmConfigs.MotionMagicCruiseVelocity = 100;
    endeffectorRotatemmConfigs.MotionMagicAcceleration = 100;
    endeffectorRotatemmConfigs.MotionMagicJerk = 100;

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
