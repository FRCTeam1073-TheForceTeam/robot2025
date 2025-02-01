// Copybottom (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TooManyListenersException;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
  private final double topKP = 0.1;
  private final double topKI = 0.0;
  private final double topKD = 0.0;
  private final double topKV = 0.0;
  private final double bottomKP = 0.1;
  private final double bottomKI = 0.0;
  private final double bottomKD = 0.0;
  private final double bottomKV = 0.0;

  private double load;
  private double bottomLoad;
  private double topLoad;
  private double velocity;
  private double topObservedVelocity;
  private double position;
  private boolean brakeMode;
  private boolean isAtZero;

  private TalonFX topLiftMotor, bottomLiftMotor;
  private VelocityVoltage topLiftMotorVelocityVoltage, bottomLiftMotorVelocityVoltage;
  private PositionVoltage topLiftMotorPositionVoltage, bottomLiftMotorPositionVoltage;
  private DigitalInput climberLiftZeroSensor;
  public Debouncer climberLiftDebouncer = new Debouncer(0.05);

  /* Creates a new ClimberLift. */
  public ClimberLift() {
    topLiftMotor = new TalonFX(16, kCANbus);
    bottomLiftMotor = new TalonFX(15, kCANbus);
    brakeMode = false;

    topLiftMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    bottomLiftMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    topLiftMotorPositionVoltage = new PositionVoltage(0).withSlot(0);
    bottomLiftMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    climberLiftZeroSensor = new DigitalInput(0);
    configureHardware();
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    topObservedVelocity = topLiftMotor.getVelocity().refresh().getValueAsDouble();//TODO: get meters per rotation and multiply by get value as double

    position = topLiftMotor.getPosition().refresh().getValueAsDouble();

    topLoad = topLiftMotor.getTorqueCurrent().getValueAsDouble();
    bottomLoad = bottomLiftMotor.getTorqueCurrent().getValueAsDouble();
    load = Math.max(topLoad, bottomLoad);
// could use load to determine when lift is at hard stop
    isAtZero = getIsAtZero();
    if (isAtZero){
      setZero();// if lift hits the bottom the position resets
    }
    topLiftMotor.setControl(topLiftMotorVelocityVoltage.withVelocity(velocity));

    SmartDashboard.putBoolean("[CLIMBER LIFT] isLiftAtZero", isAtZero);
    SmartDashboard.putBoolean("[CLIMBER LIFT] BrakeMode", brakeMode);
    SmartDashboard.putNumber("[CLIMBER LIFT] Position", position);
    SmartDashboard.putNumber("[CLIMBER LIFT] Top Observed velocity", topObservedVelocity);

    if(SmartDashboard.getBoolean("[CLIMBER LIFT] update", false)){
      SmartDashboard.putNumber("[CLIMBER LIFT] Velocity", velocity);
    }
  }
  
  public double getMaxLoad(){
    return load;
  }

  public double getloadThreshold(){
    return loadThreshold;
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public double getObservedVelocity(){
    return topObservedVelocity;
  }

  public void setZero(){
    position = 0.0;//TODO: make sure this actually works
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
    return climberLiftDebouncer.calculate(!climberLiftZeroSensor.get());
  }
  public void configureHardware(){

    var topLiftMotorClosedLoopConfig = new SlotConfigs();
    topLiftMotorClosedLoopConfig.withKP(topKP);
    topLiftMotorClosedLoopConfig.withKI(topKI);
    topLiftMotorClosedLoopConfig.withKD(topKD);
    topLiftMotorClosedLoopConfig.withKV(topKV);

    var error = topLiftMotor.getConfigurator().apply(topLiftMotorClosedLoopConfig, 0.5);

    var bottomLiftMotorClosedLoopConfig = new SlotConfigs();
    bottomLiftMotorClosedLoopConfig.withKP(bottomKP);
    bottomLiftMotorClosedLoopConfig.withKI(bottomKI);
    bottomLiftMotorClosedLoopConfig.withKD(bottomKD);
    bottomLiftMotorClosedLoopConfig.withKV(bottomKV);

    error = bottomLiftMotor.getConfigurator().apply(bottomLiftMotorClosedLoopConfig, 0.5);

    var topLiftMotorConfig = new TalonFXConfiguration();//TODO: make sure config matches physical robot
    topLiftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    topLiftMotor.getConfigurator().apply(topLiftMotorConfig);

    //var bottomLiftMotorConfig = new TalonFXConfiguration();
    //bottomLiftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //bottomLiftMotor.getConfigurator().apply(bottomLiftMotorConfig);

    topLiftMotor.setNeutralMode(NeutralModeValue.Coast);
    bottomLiftMotor.setNeutralMode(NeutralModeValue.Coast);//TODO: consider changing brakemode
    //TODO: make sure to test ungeared setup before gearing
    bottomLiftMotor.setControl(new Follower(topLiftMotor.getDeviceID(), false));

    topLiftMotor.setPosition(0);
    bottomLiftMotor.setPosition(0);

    System.out.println("Climber Configured");
  }
}
