// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberClaw extends SubsystemBase {
  private final String kCANbus = "rio";

  /** Creates a new ClimberClaw. */
  private final double gearRatio = 8;
  private final double sprocketDiameter = 0.054864;
  private final double leftMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double rightMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double leftLoadThreshold = 0;
  private final double rightLoadThreshold = 0;
  private final double leftKP = 0.1;
  private final double leftKI = 0.0;
  private final double leftKD = 0.0;
  private final double leftKV = 0.0;
  private final double rightKP = 0.1;
  private final double rightKI = 0.0;
  private final double rightKD = 0.0;
  private final double rightKV = 0.0;

  private double leftVelocity;
  private double rightVelocity;
  private double leftObservedVelocity;
  private double rightObservedVelocity;
  private double leftLoad;
  private double rightLoad;
  private boolean brakeMode;
  private boolean cageDetected;
  private DigitalInput cageDetectorSensor;

  private TalonFX leftClawMotor, rightClawMotor;
  private VelocityVoltage leftClawMotorVelocityVoltage, rightClawMotorVelocityVoltage;
  private PositionVoltage leftClawMotorPositionVoltage, rightClawMotorPositionVoltage;
  
  public Debouncer inductionSensorDebouncer = new Debouncer(0.05);

  public ClimberClaw() {
    leftVelocity = 0;
    rightVelocity = 0;
    brakeMode = false;
    cageDetected = false;
    cageDetectorSensor = new DigitalInput(1);

    leftClawMotor = new TalonFX(17, kCANbus);
    rightClawMotor = new TalonFX(18, kCANbus);

    leftClawMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightClawMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftClawMotorPositionVoltage = new PositionVoltage(0).withSlot(0);
    rightClawMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftObservedVelocity = leftClawMotor.getVelocity().refresh().getValueAsDouble() * leftMetersPerRotation;
    rightObservedVelocity = rightClawMotor.getVelocity().refresh().getValueAsDouble() * rightMetersPerRotation;

    leftLoad = leftClawMotor.getTorqueCurrent().getValueAsDouble();
    rightLoad = rightClawMotor.getTorqueCurrent().getValueAsDouble();

    cageDetected = getCageDetected();
    leftClawMotor.setControl(leftClawMotorVelocityVoltage.withVelocity(leftVelocity/leftMetersPerRotation));//TODO: test these measurements
    rightClawMotor.setControl(rightClawMotorVelocityVoltage.withVelocity(rightVelocity/rightMetersPerRotation));

    SmartDashboard.putBoolean("[CLIMBER CLAW] isCageThere", cageDetected);
    SmartDashboard.putBoolean("[CLIMBER CLAW] brake mode", brakeMode);
    SmartDashboard.putNumber("[CLIMBER CLAW] left observed velocity", leftObservedVelocity);
    SmartDashboard.putNumber("[CLIMBER CLAW] right observed velocity", rightObservedVelocity);
    
    if(SmartDashboard.getBoolean("[CLIMBER CLAW] update", false)){
      leftVelocity = SmartDashboard.getNumber("[CLIMBER CLAW] left velocity", 0);
      rightVelocity = SmartDashboard.getNumber("[CLIMBER CLAW] left velocity", 0);
    }

  }

  public double getRightLoad(){
    return rightLoad;
  }

  public double getLeftLoad(){
    return leftLoad;
  }

  public double getRightLoadThreshold(){
    return rightLoadThreshold;
  }

  public double getLeftLoadThreshold(){
    return leftLoadThreshold;
  }

  /**
   * velociity is set in meters per rotation
   * @param left velocity for left motor
   * @param right velocity for right motor
   */
  public void setVelocity(double left, double right){
    leftVelocity = left;
    rightVelocity = right;
  }

  public double getLeftObservedVelocity(){
    return leftObservedVelocity;
  }

  public double getRightObservedVelocity(){
    return rightObservedVelocity;
  }

  public void setZero(double zero){

  }

  /**
   * sets brakemode to brake or coast
   * @param mode boolean says true if it is set to brakemode
   */
  public void setBrakeMode(Boolean mode){
    if (mode){
      leftClawMotor.setNeutralMode(NeutralModeValue.Brake);
      rightClawMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      leftClawMotor.setNeutralMode(NeutralModeValue.Coast);
      rightClawMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakeMode = mode;
  }

  /**
   * gets current brakemode of climber claw
   * @return true if set to brake  false if set to coast
   */
  public boolean getBrakeMode(){
    return brakeMode;
  }

  public boolean getCageDetected(){
    return inductionSensorDebouncer.calculate(!cageDetectorSensor.get());
  }

  public void configureHardware(){

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
    //TODO: check correct brake mode
    leftClawMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClawMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClawMotor.setControl(new Follower(leftClawMotor.getDeviceID(), true));


    leftClawMotor.setPosition(0);
    rightClawMotor.setPosition(0);

    System.out.println("Climber Configured");
  }
}
