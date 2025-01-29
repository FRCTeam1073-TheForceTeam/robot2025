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

public class CoralElevator extends SubsystemBase {
  private final String kCANbus = "CANivore";
  private final double loadThreshold = 0.0;
  private final double leftKP = 0.1;
  private final double leftKD = 0.0;
  private final double leftKI = 0.0;
  private final double leftKV = 0.0;
  private final double rightKP = 0.1;
  private final double rightKD = 0.0;
  private final double rightKI = 0.0;
  private final double rightKV = 0.0;


  private double position;
  private double velocity;
  private double leftLoad;
  private double rightLoad;
  private double load;
  private double leftObservedVelocity;
  private boolean brakemode;
  private boolean isAtZero;

  private TalonFX leftElevatorMotor, rightElevatorMotor;
  private VelocityVoltage leftElevatorMotorVelocityVoltage, rightElevatorMotorVelocityVoltage;
  private PositionVoltage leftElevatorMotorPositionVoltage, rightElevatorMotorPositionVoltage;
  private DigitalInput elevatorLeftZeroSensor;
  public Debouncer coralElevatorDebouncer = new Debouncer(0.05);


  public CoralElevator() {
    brakemode = false;

    leftElevatorMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightElevatorMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftElevatorMotorPositionVoltage = new PositionVoltage(0).withSlot(0);
    rightElevatorMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    elevatorLeftZeroSensor = new DigitalInput(0);//TODO change channel
    //configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftObservedVelocity = leftElevatorMotor.getVelocity().refresh().getValueAsDouble();

    position = leftElevatorMotor.getPosition().refresh().getValueAsDouble();

    leftLoad = leftElevatorMotor.getTorqueCurrent().getValueAsDouble();
    rightLoad = rightElevatorMotor.getTorqueCurrent().getValueAsDouble();

    load = Math.max(leftLoad, rightLoad);
    isAtZero = isCoralElevatorAtBottom();

    if(isAtZero){
      setZero();
    }

    leftElevatorMotor.setControl(leftElevatorMotorVelocityVoltage.withVelocity(velocity));

    SmartDashboard.putBoolean("[CORAL ELEVATOR] is elevator at zero", isAtZero);
    SmartDashboard.putBoolean("[CORAL ELEVATOR] brake mode", brakemode);
    SmartDashboard.putNumber("[CORAL ELEVATOR] position", position);
    SmartDashboard.putNumber("[CORAL ELEVATOR] left observed velocity", leftObservedVelocity);

    if(SmartDashboard.getBoolean("[CORAL ELEVATOR] update", false)){
      SmartDashboard.putNumber("[CORAL ELEVATOR] velocity", velocity);
    }
  }

  public double getPosition(){// where motor is
    return position;
  }

  public void setZero(){
    position = 0.0;//TODO test these values
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
  }

  public void setVelocity(double velocity){
    this.velocity = velocity;
  }

  public boolean isCoralElevatorAtBottom(){
    if (position == 0.0){
      return true;
    }
    return false;
  }


  public void setPosition(double position){
    this.position = position;
  }

  public double getMaxLoad(){
    return load;
  }

  public boolean getBrakeMode(){
    return brakemode;
  }

  public boolean getIsAtZero(){
    return isAtZero;
  }

  public void setBrakeMode(boolean mode){
    if (mode){
      leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
      rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakemode = mode;
  }

  public double getloadThreshold(){
    return loadThreshold;
  }

  public double getObservedVelocity(){
    return leftObservedVelocity;
  }

  public void configureHardware(){

    var leftElevatorMotorClosedLoopConfig = new SlotConfigs();
    leftElevatorMotorClosedLoopConfig.withKP(leftKP);
    leftElevatorMotorClosedLoopConfig.withKI(leftKI);
    leftElevatorMotorClosedLoopConfig.withKD(leftKD);
    leftElevatorMotorClosedLoopConfig.withKV(leftKV);

    var error = leftElevatorMotor.getConfigurator().apply(leftElevatorMotorClosedLoopConfig, 0.5);

    var rightElevatorMotorClosedLoopConfig = new SlotConfigs();
    rightElevatorMotorClosedLoopConfig.withKP(rightKP);
    rightElevatorMotorClosedLoopConfig.withKI(rightKI);
    rightElevatorMotorClosedLoopConfig.withKD(rightKD);
    rightElevatorMotorClosedLoopConfig.withKV(rightKV);

    error = rightElevatorMotor.getConfigurator().apply(rightElevatorMotorClosedLoopConfig, 0.5);

    var leftElevatorMotorConfig = new TalonFXConfiguration();//TODO check configs with robots
    leftElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfig);

    leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));

    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    System.out.println("yay the coral Elevator was configured");
  }
}