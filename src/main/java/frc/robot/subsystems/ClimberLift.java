// Copybottom (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TooManyListenersException;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberLift extends SubsystemBase {
  private final String kCANbus = "rio";
  private final double loadThreshold = 0;
  private final double topKP = 0.2;
  private final double topKI = 0.0;
  private final double topKD = 0.0;
  private final double topKV = 0.12;


  private double load = 0.0;
  private double bottomLoad = 0.0;
  private double topLoad = 0.0;
  private double velocity = 0.0;
  private double position = 0.0;
  private double maxPosition = 67.0;
  private boolean brakeMode = false;
  private boolean isAtZero = false;
  private double commandedVelocity = 0.0;
  private boolean velocityMode = true;

  private TalonFX topLiftMotor, bottomLiftMotor;
  private VelocityVoltage topLiftMotorVelocityVoltage;
  private PositionVoltage topLiftMotorPositionVoltage;

  private DigitalInput climberLiftZeroSensor;
  public Debouncer climberLiftDebouncer = new Debouncer(0.05);

  /* Creates a new ClimberLift. */
  public ClimberLift() {
    topLiftMotor = new TalonFX(16, kCANbus);
    bottomLiftMotor = new TalonFX(15, kCANbus);
    brakeMode = true;

    /// Controller options for lift:
    topLiftMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    topLiftMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    climberLiftZeroSensor = new DigitalInput(0);
    configureHardware();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    // TODO: get meters per rotation and multiply by get value as double
    velocity = topLiftMotor.getVelocity().refresh().getValueAsDouble(); 
    position = topLiftMotor.getPosition().refresh().getValueAsDouble();

    topLoad = topLiftMotor.getTorqueCurrent().getValueAsDouble();
    bottomLoad = bottomLiftMotor.getTorqueCurrent().getValueAsDouble();
    load = Math.max(topLoad, bottomLoad);

    // Could also use load to determine when lift is at hard stop:
    isAtZero = climberLiftDebouncer.calculate(!climberLiftZeroSensor.get());

    // Stop driving towards the stop when we're at bottom:
    if (isAtZero){
      setZero(); // if lift hits the bottom the position resets to zero.
      if (commandedVelocity < 0.0) commandedVelocity = 0.0; // Velocity clamp at zero.
    }

    topLiftMotor.setControl(topLiftMotorVelocityVoltage.withVelocity(commandedVelocity));

    SmartDashboard.putBoolean("ClimberLift/isAtZero", isAtZero);
    SmartDashboard.putBoolean("ClimberLift/BrakeMode", brakeMode);
    SmartDashboard.putNumber("CimberLift/Position", position);
    SmartDashboard.putNumber("ClimberLift/Velocity", velocity);
    SmartDashboard.putNumber("ClimberLift/CommandedVelocity", commandedVelocity);
  }
  
  public double getMaxLoad(){
    return load;
  }

  public double getloadThreshold(){
    return loadThreshold;
  }

  public void setVelocity(double velocity) 
  {
    velocityMode = true;
    commandedVelocity = velocity;
  }

  public double getVelocity(){
    return velocity;
  }

  public void setZero(){
    position = 0.0; // TODO: make sure this actually works
    topLiftMotor.setPosition(0);
    bottomLiftMotor.setPosition(0);
  }

  public void setBrakeMode(Boolean mode){
    if (mode){
      topLiftMotor.setNeutralMode(NeutralModeValue.Brake);
      bottomLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      topLiftMotor.setNeutralMode(NeutralModeValue.Coast);
      bottomLiftMotor.setNeutralMode(NeutralModeValue.Coast);
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
    var topLiftMotorConfig = new TalonFXConfiguration();//TODO: make sure config matches physical robot
    //set to clockwise is positive so that negative commanded velocity moved lift down
    topLiftMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    topLiftMotor.getConfigurator().apply(topLiftMotorConfig);
    bottomLiftMotor.getConfigurator().apply(topLiftMotorConfig);

    var topLiftMotorClosedLoopConfig = new SlotConfigs();
    topLiftMotorClosedLoopConfig.withKP(topKP);
    topLiftMotorClosedLoopConfig.withKI(topKI);
    topLiftMotorClosedLoopConfig.withKD(topKD);
    topLiftMotorClosedLoopConfig.withKV(topKV);

    var error = topLiftMotor.getConfigurator().apply(topLiftMotorClosedLoopConfig, 0.5);
    

    // TODO: Set in brake mode normally. For testing... coast.
    topLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    bottomLiftMotor.setNeutralMode(NeutralModeValue.Brake);

    CurrentLimitsConfigs topLiftCurrentLimitsConfigs = new CurrentLimitsConfigs();
    topLiftCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(15)
                            .withSupplyCurrentLowerTime(0.25);

    topLiftMotor.getConfigurator().apply(topLiftCurrentLimitsConfigs);
    bottomLiftMotor.getConfigurator().apply(topLiftCurrentLimitsConfigs);

    //TODO: make sure to test ungeared setup before gearing
    bottomLiftMotor.setControl(new Follower(topLiftMotor.getDeviceID(), false));

    topLiftMotor.setPosition(0);
    bottomLiftMotor.setPosition(0);

    System.out.println("ClimberLift: Configured");
  }
}
