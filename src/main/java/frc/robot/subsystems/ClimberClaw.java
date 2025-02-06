// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
  // private final double leftMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  // private final double rightMetersPerRotation = sprocketDiameter * Math.PI / gearRatio;
  private final double leftMetersPerRotation = 1;
  private final double rightMetersPerRotation = 1;
  private final double loadThreshold = 5.0; // TODO: Current limiting
  private final double leftKP = 0.3;
  private final double leftKI = 0.0;
  private final double leftKD = 0.0;
  private final double leftKV = 0.12;

  private double velocity = 0.0;
  private double load = 0.0;
  private boolean brakeMode = false;
  private double position;
  private boolean cageDetected = false;
  private DigitalInput cageDetectorSensor;
  private double commandedVelocity = 0.0;

  private TalonFX leftClawMotor, rightClawMotor;
  // Control loops for the leader motor:
  private VelocityVoltage leftClawMotorVelocityVoltage;
  private PositionVoltage leftClawMotorPositionVoltage;
  
  public Debouncer inductionSensorDebouncer = new Debouncer(0.05);

  public ClimberClaw() {
    velocity = 0;
    brakeMode = false;
    cageDetected = false;
    cageDetectorSensor = new DigitalInput(1);
    leftClawMotor = new TalonFX(17, kCANbus);
    rightClawMotor = new TalonFX(18, kCANbus);
    

    configureHardware();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

    double leftObservedVelocity = leftClawMotor.getVelocity().refresh().getValueAsDouble() * leftMetersPerRotation;
    double rightObservedVelocity = rightClawMotor.getVelocity().refresh().getValueAsDouble() * rightMetersPerRotation;
    velocity = (leftObservedVelocity + rightObservedVelocity) / 2.0;

    double leftLoad = Math.abs(leftClawMotor.getTorqueCurrent().getValueAsDouble());
    double rightLoad = Math.abs(rightClawMotor.getTorqueCurrent().getValueAsDouble());
    load = leftLoad + rightLoad;

    cageDetected = inductionSensorDebouncer.calculate(!cageDetectorSensor.get());

    // TODO: test these measurements
    leftClawMotor.setControl(leftClawMotorVelocityVoltage.withVelocity(commandedVelocity/leftMetersPerRotation));
    
    SmartDashboard.putBoolean("ClimberClaw/isCageThere", cageDetected);
    SmartDashboard.putBoolean("ClimberClaw/Brake mode", brakeMode);
    SmartDashboard.putNumber("ClimberClaw/commanded velocity", commandedVelocity);
    SmartDashboard.putNumber("ClimberClaw/velocity", velocity);
    SmartDashboard.putNumber("ClimberClaw/load", load);

  }

  public double getLoad(){
    return load;
  }

  public double getLoadThreshold(){
    return loadThreshold;
  }

  /**
   * velociity is set in meters per rotation
   * @param left velocity for left motor
   * @param right velocity for right motor
   */
  public void setVelocity(double velocity)
  {
    commandedVelocity = velocity;
  }

  public double getVelocity(){
    return velocity;
  }

  public void setZero(double zero){
    leftClawMotor.setPosition(0.0);
    rightClawMotor.setPosition(0.0);
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
    return cageDetected;
  }

  public void setIsAtZero(boolean zero){
    position = 0;
  }

  public boolean getIsAtZero(){
    return position == 0;
  }

  public void configureHardware(){
    TalonFXConfiguration clawConfigs = new TalonFXConfiguration();
    clawConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leftClawMotor.getConfigurator().apply(clawConfigs);
    rightClawMotor.getConfigurator().apply(clawConfigs);

    leftClawMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    // TODO: Position control slot assignment?
    leftClawMotorPositionVoltage = new PositionVoltage(0).withSlot(0);


    var leftClawMotorClosedLoopConfig = new SlotConfigs();
    leftClawMotorClosedLoopConfig.withKP(leftKP);
    leftClawMotorClosedLoopConfig.withKI(leftKI);
    leftClawMotorClosedLoopConfig.withKD(leftKD);
    leftClawMotorClosedLoopConfig.withKV(leftKV);

    var error = leftClawMotor.getConfigurator().apply(leftClawMotorClosedLoopConfig, 0.5);

 
    //TODO: check correct brake mode
    leftClawMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClawMotor.setNeutralMode(NeutralModeValue.Brake);


    CurrentLimitsConfigs clawCurrentLimitsConfigs = new CurrentLimitsConfigs();
    clawCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(10)
                            .withSupplyCurrentLimit(6)
                            .withSupplyCurrentLowerTime(0.25);

    leftClawMotor.getConfigurator().apply(clawCurrentLimitsConfigs);
    rightClawMotor.getConfigurator().apply(clawCurrentLimitsConfigs);


    // Claw motors are inverted follower configuration.
    rightClawMotor.setControl(new Follower(leftClawMotor.getDeviceID(), false));

    // Start at zero:
    leftClawMotor.setPosition(0);
    rightClawMotor.setPosition(0);

    System.out.println("Climber Configured");
  }
}
