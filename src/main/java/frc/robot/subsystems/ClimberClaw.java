// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberClaw extends SubsystemBase {
  private final String kCANbus = "CANivore";

  /** Creates a new ClimberClaw. */
  private final double gearRatio = 8;
  private final double sprocketDiameter = 0.054864;
  private final double leftMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double rightMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double leftKP = 0.1;
  private final double leftKI = 0.0;
  private final double leftKD = 0.0;
  private final double leftKV = 0.0;
  private final double rightKP = 0.1;
  private final double rightKI = 0.0;
  private final double rightKD = 0.0;
  private final double rightKV = 0.0;

  private double velocity;
  private double load;
  private boolean brakeMode;
  private boolean cageDetected;
  private DigitalInput cageDetectorSensor;

  private TalonFX leftClawMotor, rightClawMotor;
  private VelocityVoltage leftClawMotorVelocityVoltage, rightClawMotorVelocityVoltage;
  private PositionVoltage leftClawMotorPositionVoltage, rightClawMotorPositionVoltage;
  
  public Debouncer inductionSensorDebouncer = new Debouncer(0.05);

  public ClimberClaw() {
    velocity = 0;
    brakeMode = false;
    cageDetected = false;
    cageDetectorSensor = new DigitalInput(4);

    /*leftClawMotor = new TalonFX(-1, kCANbus);
    rightClawMotorf = new TalonFX(-1, kCANbus);*/

    leftClawMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightClawMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftClawMotorPositionVoltage = new PositionVoltage(0).withSlot(0);
    rightClawMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    //configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cageDetected = getCageDetected();
    SmartDashboard.putBoolean("isCageThere", cageDetected);
  }

  public double getLoad(){
    return load;
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public double getVelocity(){
    return velocity;
  }

  public void setZero(double zero){

  }
  
  public void setBrakeMode(Boolean mode){
    brakeMode = mode;
  }


  public boolean getBrakeMode(){
    return brakeMode;
  }

  public boolean getCageDetected(){
    return inductionSensorDebouncer.calculate(!cageDetectorSensor.get());
  }

  /*@Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("OI");
    builder.addBooleanProperty("is CageDetected", this::getCageDetected, null);
  }*/
  /*public void configureHardware(){

    var leftClawMotorClosedLoopConfig = new SlotConfigs();
    leftClawMotorClosedLoopConfig.withKP(leftKP);
    leftClawMotorClosedLoopConfig.withKI(leftKI);
    leftClawMotorClosedLoopConfig.withKD(leftKD);
    leftClawMotorClosedLoopConfig.withKV(leftKV);

    var error = leftClawMotor.getConfigurator().apply(leftClawMotorClosedLoopConfig, 0.5);

    var rightClawMotorClosedLoopConfig = new SlotConfigs();
    rightClawMotorClosedLoopConfig.withKP(rightKP);
    rightClawMotorClosedLoopConfig.withKI(rightKI);
    rightClawMotorClosedLoopConfig.withKD(rightKD);
    rightClawMotorClosedLoopConfig.withKV(rightKV);

    error = rightClawMotor.getConfigurator().apply(rightClawMotorClosedLoopConfig, 0.5);

    var leftClawMotorConfig = new TalonFXConfiguration();//TODO: make sure config matches physical robot
    leftClawMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftClawMotor.getConfigurator().apply(leftClawMotorConfig);
    
    var rightClawMotorConfig = new TalonFXConfiguration();
    rightClawMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightClawMotor.getConfigurator().apply(rightClawMotorConfig);

    leftClawMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClawMotor.setNeutralMode(NeutralModeValue.Brake);

    leftClawMotor.setPosition(0);
    rightClawMotor.setPosition(0);

    System.out.println("Climber Configured");
  }*/
}
