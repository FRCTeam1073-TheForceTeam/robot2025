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

  private final double collectPosKP = 0.15; //TODO change these values
  private final double collectPosKI = 0;
  private final double collectPosKD = 0;
  private final double collectPosKV = 0.12;

  private final double rotateVelKP = 0.2;
  private final double rotateVelKI = 0.01;
  private final double rotateVelKD = 0.0;
  private final double rotateVelKV = 0.12;
  private final double rotateVelKA = 0.01;

  private final double rotatePosKP = 0.2;
  private final double rotatePosKI = 0.01;
  private final double rotatePosKD = 0.0;
  private final double rotatePosKV = 0.12;
  private final double rotatePosKA = 0.01;

  private double collectVel;
  private double commandedCollectVel;
  private double commandedCollectPos;
  private double collectPos;
  private double collectLoad;

  private double rotateVel;
  private double commandedRotateVel;
  private double commandedRotatePos;
  private double rotatePos;
  private double rotateLoad;

  private boolean hasAlgae;
  private boolean collectBrakeMode = true;
  private boolean rotateBrakeMode = true;
  private boolean isUp = true;
  private boolean velocityMode = true;

  private TalonFX collectMotor;
  private TalonFX rotateMotor;
  private VelocityVoltage collectVelocityVoltage;
  private MotionMagicVoltage collectPositionVoltage;
  private VelocityVoltage rotateVelocityVoltage;
  private MotionMagicVoltage rotatePositionController;


  public AlgaeClaw() {
    collectMotor = new TalonFX(25);
    rotateMotor = new TalonFX(26);
    collectBrakeMode = true;
    rotateBrakeMode = true;
    velocityMode = true;
    isUp = true;

    commandedRotateVel = 0.0;
    commandedCollectVel = 0.0;

    collectVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rotateVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rotatePositionController = new MotionMagicVoltage(0).withSlot(1);
    collectPositionVoltage = new MotionMagicVoltage(0).withSlot(1);

    configureHardware();
  }

  @Override
  public void periodic() {
    collectVel = collectMotor.getVelocity().getValueAsDouble(); 
    collectPos = collectMotor.getPosition().getValueAsDouble(); 
    collectLoad = collectMotor.getTorqueCurrent().getValueAsDouble();

    rotateVel = rotateMotor.getVelocity().getValueAsDouble(); 
    rotatePos = rotateMotor.getPosition().getValueAsDouble();
    rotateLoad = rotateMotor.getTorqueCurrent().getValueAsDouble();

    commandedCollectPos = commandedCollectVel * 0.5; //calculating collect position based on velocity and time
    collectMotor.setControl(collectPositionVoltage.withPosition(commandedCollectPos).withSlot(1));


    if (rotatePos >= rotateMaxPos){
      commandedRotateVel = Math.min(commandedRotateVel, 0); 
    }
    if (rotatePos <= rotateMinPos){
      commandedRotateVel = Math.max(commandedRotateVel, 0);
    }

    if(rotatePos >= 8.7) {
      isUp = false;
    }
    else if(rotatePos < 8.7) {
      isUp = true;
    }

    if(velocityMode) {
      rotateMotor.setControl(rotateVelocityVoltage.withVelocity(commandedRotateVel).withSlot(0));
    }
    else {
      rotateMotor.setControl(rotatePositionController.withPosition(commandedRotatePos).withSlot(1));
    }

    SmartDashboard.putNumber("AlgaeClaw/Collect Velocity", collectVel);
    SmartDashboard.putNumber("AlgaeClaw/Collect Commanded Velocity", commandedCollectVel);
    SmartDashboard.putNumber("AlgaeClaw/Collect Motor Load", collectLoad);
    SmartDashboard.putBoolean("AlgaeClaw/Collect Brake Mode", !collectBrakeMode);
    SmartDashboard.putNumber("AlgaeClaw/Rotate Velocity", rotateVel);
    SmartDashboard.putNumber("AlgaeClaw/Rotate Commanded Velocity", commandedRotateVel);
    SmartDashboard.putNumber("AlgaeClaw/Rotate Position", rotatePos);
    SmartDashboard.putNumber("AlgaeClaw/Rotate Motor Load", rotateLoad);
    SmartDashboard.putBoolean("AlgaeClaw/Rotate Break Mode", !rotateBrakeMode);
    SmartDashboard.putBoolean("AlgaeClaw/Has Algae", hasAlgae);
    SmartDashboard.putBoolean("AlgaeClaw/Is Up", isUp);
  }

  public void setCollectorVel(double newVel){
    commandedCollectVel = newVel;
  }

  public void setRotatorVel(double newVel){
    velocityMode = true;
    commandedRotateVel = newVel;
  }

  public void setRotatorPos(double position) {
    velocityMode = false;
    commandedRotatePos = position;
  }

  public void setIsUp(boolean state) {
    isUp = state;
  }

  public void zeroRotator(){
    rotatePos = 0.0;
    rotateMotor.setPosition(0);
  }

  public void setCollectorBrakeMode(boolean mode){
    if(mode){
      collectMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      collectMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    collectBrakeMode = mode;
  }

  public void setRotateBrakeMode(boolean mode){
    if(mode){
      rotateMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      rotateMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    rotateBrakeMode = mode;
  }

  public double getCollectorVelocity(){
    return collectVel;
  }

  public double getCollectorLoad(){
    return collectLoad;
  }

  public double getRotatorVelocity(){
    return rotateVel;
  }

  public double getRotatorPosition(){
    return rotatePos;
  }

  public double getRotatorLoad(){
    return rotateLoad;
  }

  public boolean getHasAlgae(){
    return hasAlgae;
  }

  public boolean getIsAtZero(){
    return rotatePos <= .01;
  }

  public boolean getIsUp(){
    return isUp;
  }

  public boolean getRotatorBrakeMode(){
    return rotateBrakeMode;
  }

  public boolean getCollectorBrakeMode(){
    return collectBrakeMode;
  }

  public void configureHardware()
  {
    var rotateMotorConfig = new TalonFXConfiguration();
    var collectMotorConfig = new TalonFXConfiguration();
    rotateMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    collectMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var rotatemmConfigs = rotateMotorConfig.MotionMagic;
    rotatemmConfigs.MotionMagicCruiseVelocity = 75; //TODO: tune numbers
    rotatemmConfigs.MotionMagicAcceleration = 100;
    rotatemmConfigs.MotionMagicJerk = 800;

    var collectmmConfigs = rotateMotorConfig.MotionMagic;
    collectmmConfigs.MotionMagicCruiseVelocity = 75; //TODO: tune numbers
    collectmmConfigs.MotionMagicAcceleration = 100;
    collectmmConfigs.MotionMagicJerk = 800;
    
    var collectMotorClosedLoop0Config = collectMotorConfig.Slot0;
    var collectMotorClosedLoop1Config = collectMotorConfig.Slot1;
    var rotateMotorClosedLoop0Config = rotateMotorConfig.Slot0;
    var rotateMotorClosedLoop1Config = rotateMotorConfig.Slot1;

    collectMotorClosedLoop0Config.withKP(collectMotorKP);
    collectMotorClosedLoop0Config.withKI(collectMotorKI);
    collectMotorClosedLoop0Config.withKD(collectMotorKD);
    collectMotorClosedLoop0Config.withKV(collectMotorKV);

    collectMotorClosedLoop1Config.withKP(collectPosKP);
    collectMotorClosedLoop1Config.withKI(collectPosKI);
    collectMotorClosedLoop1Config.withKD(collectPosKD);
    collectMotorClosedLoop1Config.withKV(collectPosKV);

    rotateMotorClosedLoop0Config.withKP(rotateVelKP);
    rotateMotorClosedLoop0Config.withKI(rotateVelKI);
    rotateMotorClosedLoop0Config.withKD(rotateVelKD);
    rotateMotorClosedLoop0Config.withKV(rotateVelKV);
    rotateMotorClosedLoop0Config.withKA(rotateVelKA);

    rotateMotorClosedLoop1Config.withKP(rotatePosKP);
    rotateMotorClosedLoop1Config.withKI(rotatePosKI);
    rotateMotorClosedLoop1Config.withKD(rotatePosKD);
    rotateMotorClosedLoop1Config.withKV(rotatePosKV);
    rotateMotorClosedLoop1Config.withKA(rotatePosKA);

    rotateMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    collectMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    collectMotor.getConfigurator().apply(collectMotorConfig, 0.5);
    rotateMotor.getConfigurator().apply(rotateMotorConfig, 0.5);

    collectMotor.setNeutralMode(NeutralModeValue.Coast);
    rotateMotor.setNeutralMode(NeutralModeValue.Coast);
   
    collectMotor.setPosition(0);
    rotateMotor.setPosition(0);

    System.out.println("Algae Collector & Rotator: Configured");
  }
}
