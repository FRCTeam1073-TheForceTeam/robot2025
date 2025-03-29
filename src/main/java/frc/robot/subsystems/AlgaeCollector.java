// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeCollector extends SubsystemBase {

  private final double collectMotorKP = 0.2; //TODO change these values
  private final double collectMotorKI = 0;
  private final double collectMotorKD = 0.02;
  private final double collectMotorKV = 0.12;

  private final double collectPosKP = 0.15; //TODO change these values
  private final double collectPosKI = 0;
  private final double collectPosKD = 0;
  private final double collectPosKV = 0.12;

  private double collectVel;
  private double commandedCollectVel;
  private double commandedCollectPos;
  private double collectPos;
  private double collectLoad;
  private double collectTemp;


  private boolean hasAlgae;
  private boolean collectBrakeMode = true;

  private TalonFX collectMotor;
  private VelocityVoltage collectVelocityVoltage;
  private PositionVoltage collectPositionVoltage;
  private LinearFilter filter;

  public AlgaeCollector() {
    filter = LinearFilter.singlePoleIIR(0.5, 0.02);
    collectMotor = new TalonFX(25);
    collectBrakeMode = true;

    commandedCollectVel = 0.0;

    collectVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    collectPositionVoltage = new PositionVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() {
    collectTemp = collectMotor.getDeviceTemp().getValueAsDouble();
    collectVel = collectMotor.getVelocity().getValueAsDouble(); 
    collectPos = collectMotor.getPosition().getValueAsDouble(); 
    collectLoad = collectMotor.getTorqueCurrent().getValueAsDouble();
    collectLoad = filter.calculate(collectLoad);

    commandedCollectPos = commandedCollectPos + (commandedCollectVel * 0.02); //calculating collect position based on velocity and time
    collectMotor.setControl(collectPositionVoltage.withPosition(commandedCollectPos).withSlot(0));

    // if (rotatePos >= rotateMaxPos){
    //   commandedRotateVel = Math.min(commandedRotateVel, 0); 
    // }
    // if (rotatePos <= rotateMinPos){
    //   commandedRotateVel = Math.max(commandedRotateVel, 0);
    // }

    // if(rotatePos >= 8.7) {
    //   isUp = false;
    // }
    // else if(rotatePos < 8.7) {
    //   isUp = true;
    // }

    // if(velocityMode) {
    //   rotateMotor.setControl(rotateVelocityVoltage.withVelocity(commandedRotateVel).withSlot(0));
    // }
    // else {
    //   rotateMotor.setControl(rotatePositionController.withPosition(commandedRotatePos).withSlot(1));
    // }

    SmartDashboard.putNumber("AlgaeClaw/Collect Velocity", collectVel);
    SmartDashboard.putNumber("AlgaeClaw/Collect Position", collectPos);
    SmartDashboard.putNumber("AlgaeClaw/Collect Commanded Velocity", commandedCollectVel);
    SmartDashboard.putNumber("AlgaeClaw/Collect Motor Load", collectLoad);
    SmartDashboard.putBoolean("AlgaeClaw/Collect Brake Mode", !collectBrakeMode);
    SmartDashboard.putBoolean("AlgaeClaw/Has Algae", hasAlgae);
    SmartDashboard.putNumber("AlgaeClaw/Algae Collect Motor Temp", collectTemp);
  }

  public void setCollectorVel(double newVel){
    commandedCollectVel = newVel;
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

  public double getCollectorVelocity(){
    return collectVel;
  }

  public double getCollectorLoad(){
    return collectLoad;
  }

  public boolean getHasAlgae(){
    return hasAlgae;
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
    
    var collectMotorClosedLoop0Config = collectMotorConfig.Slot0;
    var rotateMotorClosedLoop0Config = rotateMotorConfig.Slot0;
    var rotateMotorClosedLoop1Config = rotateMotorConfig.Slot1;

    collectMotorClosedLoop0Config.withKP(collectMotorKP);
    collectMotorClosedLoop0Config.withKI(collectMotorKI);
    collectMotorClosedLoop0Config.withKD(collectMotorKD);
    collectMotorClosedLoop0Config.withKV(collectMotorKV);

    rotateMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    collectMotorConfig.CurrentLimits.withSupplyCurrentLimitEnable(true)
    .withSupplyCurrentLimit(15)
    .withSupplyCurrentLowerTime(0.25);

    collectMotor.getConfigurator().apply(collectMotorConfig, 0.5);

    collectMotor.setNeutralMode(NeutralModeValue.Brake);
   
    collectMotor.setPosition(0);

    System.out.println("Algae Collector: Configured");
  }
}
