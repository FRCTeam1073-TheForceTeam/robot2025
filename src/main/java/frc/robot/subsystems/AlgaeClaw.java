// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeClaw extends SubsystemBase {

  private final double rotateMinPos = 0;
  private final double rotateMaxPos = 28.476;

  private final double collectMotorKP = 0.15; //TODO change these values
  private final double collectMotorKI = 0;
  private final double collectMotorKD = 0;
  private final double collectMotorKV = 0.12;

  private final double rotateVelKP = 0.15;
  private final double rotateVelKI = 0;
  private final double rotateVelKD = 0;
  private final double rotateVelKV = 0.12;

  private final double rotatePosKP = 0.15;
  private final double rotatePosKI = 0;
  private final double rotatePoKD = 0;
  private final double rotatePosKV = 0.12;

  private double collectVel;
  private double commandedCollectVel;
  private double collectLoad;

  private double rotateVel;
  private double commandedRotateVel;
  private double rotatePos;
  private double rotateLoad;

  private boolean hasAlgae;
  private boolean collectBrakeMode = true;
  private boolean rotateBrakeMode = true;
  private boolean endeffectorIsUp = true;
  private boolean velocityMode = true;

  private TalonFX collectMotor;
  private TalonFX rotateMotor;
  private VelocityVoltage collectVelocityVoltage;
  private VelocityVoltage rotateVelocityVoltage;
  private MotionMagicVoltage rotatePositionController;


  public AlgaeClaw() {
    collectMotor = new TalonFX(25);
    rotateMotor = new TalonFX(26);
    collectBrakeMode = true;
    rotateBrakeMode = true;
    velocityMode = true;

    commandedRotateVel = 0.0;
    commandedCollectVel = 0.0;

    collectVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rotateVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rotatePositionController = new MotionMagicVoltage(0).withSlot(1);

    configureHardware();
  }

  @Override
  public void periodic() {
    collectVel = collectMotor.getVelocity().getValueAsDouble(); 
    collectLoad = collectMotor.getTorqueCurrent().getValueAsDouble();

    rotateVel = rotateMotor.getVelocity().getValueAsDouble(); 
    rotatePos = rotateMotor.getPosition().getValueAsDouble();
    rotateLoad = rotateMotor.getTorqueCurrent().getValueAsDouble();

    if (rotatePos >= rotateMaxPos){
      commandedRotateVel = Math.min(commandedRotateVel, 0); //TODO: this is where we need the with slot
    }
    if (rotatePos <= rotateMinPos){
      commandedRotateVel = Math.max(commandedRotateVel, 0);
    }

    collectMotor.setControl(collectVelocityVoltage.withVelocity(commandedCollectVel));
    rotateMotor.setControl(rotateVelocityVoltage.withVelocity(commandedRotateVel));

    SmartDashboard.putNumber("[AlgaeClaw]/ algae collect velocity", collectVel);
    SmartDashboard.putNumber("[AlgaeClaw]/ algae collect commanded velocity", commandedCollectVel);
    SmartDashboard.putNumber("[AlgaeClaw]/ endeffector rotate velocity", rotateVel);
    SmartDashboard.putNumber("[AlgaeClaw]/ endeffector rotate position", rotatePos);
    SmartDashboard.putNumber("[AlgaeClaw]/ endeffector rotate commanded velocity", commandedRotateVel);
    SmartDashboard.putBoolean("[AlgaeClaw]/ has algae", hasAlgae);
  }

  public void setAlgaeCollectorVel(double newVel){
    commandedCollectVel = newVel;
  }

  public void setEndeffectorRotatorVel(double newVel){
    commandedRotateVel = newVel;
  }

  public void zeroEndeffectorRotator(){
    if(!this.getIsEndeffectorRotatorAtZero()){
      commandedRotateVel = -1;
    }
  }

  public void changeEndeffectorIsUp(){
    endeffectorIsUp = !endeffectorIsUp;
  }

  public void setAlgaeCollectorMotorBrakeMode(boolean mode){
    if(mode){
      collectMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      collectMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakeMode = mode;
  }

  public void setEndeffectorRotateMotorBrakeMode(boolean mode){
    if(mode){
      rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      rotateMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    endeffectorRotateBrakeMode = mode;
  }

  public double getAlgaeCollectorVelocity(){
    return collectVel;
  }

  public double getAlgaeCollectorLoad(){
    return collectLoad;
  }

  public double getEndeffectorRotatorVelocity(){
    return rotateVel;
  }

  public double getEndeffectorRotatorPosition(){
    return rotatePos;
  }

  public double getEndeffectorRotatorLoad(){
    return rotateLoad;
  }

  public boolean getHasAlgae(){
    return hasAlgae;
  }

  public boolean getIsEndeffectorRotatorAtZero(){
    return rotatePos <= .01;
  }

  public boolean getIsEndeffectorRotatorUp(){
    return endeffectorIsUp;
  }

  public boolean getEndeffectorRotatorBrakeMode(){
    return endeffectorRotateBrakeMode;
  }

  public boolean getAlgaeCollectorBrakeMode(){
    return brakeMode;
  }

  public void configureHardware()
  {
    var motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    collectMotor.getConfigurator().apply(motorConfig);
    rotateMotor.getConfigurator().apply(motorConfig);

    var algaeCollectmmConfigs = motorConfig.MotionMagic;
    var endeffectorRotatemmConfigs = motorConfig.MotionMagic;
    algaeCollectmmConfigs.MotionMagicCruiseVelocity = 100;
    algaeCollectmmConfigs.MotionMagicAcceleration = 100;
    algaeCollectmmConfigs.MotionMagicJerk = 100;
    endeffectorRotatemmConfigs.MotionMagicCruiseVelocity = 100;
    endeffectorRotatemmConfigs.MotionMagicAcceleration = 100;
    endeffectorRotatemmConfigs.MotionMagicJerk = 100;

    var algaeCollectMotorClosedLoopConfig = new SlotConfigs();
    algaeCollectMotorClosedLoopConfig.withKP(collectMotorKP);
    algaeCollectMotorClosedLoopConfig.withKI(collectMotorKI);
    algaeCollectMotorClosedLoopConfig.withKD(collectMotorKD);
    algaeCollectMotorClosedLoopConfig.withKV(collectMotorKV);

    collectMotor.getConfigurator().apply(algaeCollectMotorClosedLoopConfig, 0.5);

    var endeffectorRotateMotorClosedLoopConfigZero = motorConfig.Slot0;
    endeffectorRotateMotorClosedLoopConfigZero.withKP(rotateVelKP);
    endeffectorRotateMotorClosedLoopConfigZero.withKI(rotateVelKI);
    endeffectorRotateMotorClosedLoopConfigZero.withKD(rotateVelKD);
    endeffectorRotateMotorClosedLoopConfigZero.withKV(rotateVelKV);

    rotateMotor.getConfigurator().apply(endeffectorRotateMotorClosedLoopConfigZero, .5);

    var endeffectorRotateMotorClosedLoopConfigOne = motorConfig.Slot1;
    endeffectorRotateMotorClosedLoopConfigOne.withKP(rotatePosKP);
    endeffectorRotateMotorClosedLoopConfigOne.withKI(rotatePosKI);
    endeffectorRotateMotorClosedLoopConfigOne.withKD(rotatePoKD);
    endeffectorRotateMotorClosedLoopConfigOne.withKV(rotatePosKV);

    rotateMotor.getConfigurator().apply(endeffectorRotateMotorClosedLoopConfigOne, .5);

    collectMotor.setNeutralMode(NeutralModeValue.Coast);
    rotateMotor.setNeutralMode(NeutralModeValue.Coast);

    CurrentLimitsConfigs algaeCollectMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    algaeCollectMotorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                                         .withSupplyCurrentLimit(15)
                                         .withSupplyCurrentLowerTime(0.25);
    CurrentLimitsConfigs endeffectorRotateMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    endeffectorRotateMotorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                                         .withSupplyCurrentLimit(15)
                                         .withSupplyCurrentLowerTime(0.25);

    collectMotor.getConfigurator().apply(algaeCollectMotorCurrentLimitsConfigs);
    rotateMotor.getConfigurator().apply(endeffectorRotateMotorCurrentLimitsConfigs);
   
    collectMotor.setPosition(0);
    rotateMotor.setPosition(0);

    System.out.println("AlgaeCollector & EndeffectorRotator: Configured");
  }
}
