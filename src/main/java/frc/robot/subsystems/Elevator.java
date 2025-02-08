// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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

public class Elevator extends SubsystemBase {
  
  private final String kCANbus = "rio";
  private final double loadThreshold = 0;
  private final double leftKP = 0.1; // TODO: tune values
  private final double leftKI = 0.0;
  private final double leftKD = 0.0;
  private final double leftKV = 0.0;


  private double load = 0.0;
  private double leftLoad = 0.0;
  private double rightLoad = 0.0;
  private double velocity = 0.0;
  private double position = 0.0;
  private boolean brakeMode = false;
  private boolean isAtZero = false;
  private double commandedVelocity = 0.0;
  private boolean velocityMode = true;

  private TalonFX leftMotor, rightMotor;
  private VelocityVoltage leftVelocityVoltage;
  private PositionVoltage leftPositionVoltage;

  private DigitalInput leftZeroSensor;
  private DigitalInput rightZeroSensor; 
  public Debouncer leftZeroDebouncer = new Debouncer(0.05);
  public Debouncer rightZeroDebouncer = new Debouncer(0.05);

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = new TalonFX(19, kCANbus);
    rightMotor = new TalonFX(20, kCANbus);
    brakeMode = true;

    leftVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    leftPositionVoltage = new PositionVoltage(0).withSlot(0);

    leftZeroSensor = new DigitalInput(0);
    rightZeroSensor = new DigitalInput(0);
    configureHardware();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    velocity = leftMotor.getVelocity().refresh().getValueAsDouble(); 
    position = leftMotor.getPosition().refresh().getValueAsDouble();

    leftLoad = leftMotor.getTorqueCurrent().getValueAsDouble();
    rightLoad = rightMotor.getTorqueCurrent().getValueAsDouble();
    load = Math.max(leftLoad, rightLoad);

    isAtZero = leftZeroDebouncer.calculate(!leftZeroSensor.get()); // TODO: get both zeros of L and R?

    if (isAtZero){
      setZero(); // if lift hits the bottom the position resets to zero.
      if (commandedVelocity < 0.0) commandedVelocity = 0.0; // Velocity clamp at zero.
    }

    leftMotor.setControl(leftVelocityVoltage.withVelocity(commandedVelocity));
    rightMotor.setControl(new DutyCycleOut(0.0)); // TODO: does this need to be set to 0 if its a follower

    SmartDashboard.putBoolean("Elevator/isAtZero", isAtZero);
    SmartDashboard.putBoolean("Elevator/BrakeMode", brakeMode);
    SmartDashboard.putNumber("Elevator/Position", position);
    SmartDashboard.putNumber("Elevator/Velocity", velocity);
    SmartDashboard.putNumber("Elevator/CommandedVelocity", commandedVelocity);
  }

    public double getMaxLoad(){
      return load;
    }
  
    public double getloadThreshold(){
      return loadThreshold;
    }
  
    public void setVelocity(double velocity) {
      velocityMode = true;
      commandedVelocity = velocity;
    }
  
    public double getVelocity(){
      return velocity;
    }

    public void setZero(){
      position = 0.0; 
      rightMotor.setPosition(0);
      leftMotor.setPosition(0);
    }

    public void setBrakeMode(Boolean mode){
    if (mode){
      leftMotor.setNeutralMode(NeutralModeValue.Brake);
      rightMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      leftMotor.setNeutralMode(NeutralModeValue.Coast);
      rightMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakeMode = mode;
  }

  public boolean getBrakeMode(){
    return brakeMode;
  }

  public double getPosition(){
    return position;
  }

  public boolean getIsAtZero(){//button at botton of lift is hit
    return isAtZero;
  }

  public void configureHardware(){
    var leftMotorConfig = new TalonFXConfiguration();
    leftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //TODO: make sure this is the right way!!!
    leftMotor.getConfigurator().apply(leftMotorConfig);
    rightMotor.getConfigurator().apply(leftMotorConfig);

    var leftMotorClosedLoopConfig = new SlotConfigs();
    leftMotorClosedLoopConfig.withKP(leftKP);
    leftMotorClosedLoopConfig.withKI(leftKI);
    leftMotorClosedLoopConfig.withKD(leftKD);
    leftMotorClosedLoopConfig.withKV(leftKV);

    var error = leftMotor.getConfigurator().apply(leftMotorClosedLoopConfig, 0.5);

    leftMotor.setNeutralMode(NeutralModeValue.Coast);
    rightMotor.setNeutralMode(NeutralModeValue.Coast);

    CurrentLimitsConfigs leftMotorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    leftMotorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                                 .withSupplyCurrentLimit(15)
                                 .withSupplyCurrentLowerTime(0.25);

   leftMotor.getConfigurator().apply(leftMotorCurrentLimitsConfigs);
   rightMotor.getConfigurator().apply(leftMotorCurrentLimitsConfigs);

   rightMotor.setControl(new Follower(leftMotor.getDeviceID(), true)); //TODO: check to see if it needs to be inversed

   leftMotor.setPosition(0);
   rightMotor.setPosition(0);

   System.out.println("Elevator: Configured");

     }
  }
