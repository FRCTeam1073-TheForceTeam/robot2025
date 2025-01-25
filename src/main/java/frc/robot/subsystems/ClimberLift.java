// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private final String kCANbus = "CANivore";
  private final double leftKP = 0.1;
  private final double leftKI = 0.0;
  private final double leftKD = 0.0;
  private final double leftKV = 0.0;
  private final double rightKP = 0.1;
  private final double rightKI = 0.0;
  private final double rightKD = 0.0;
  private final double rightKV = 0.0;

  private double load;
  private double velocity;
  private double position;
  private boolean brakeMode;
  private boolean isAtZero;

  private TalonFX leftLiftMotor, rightLiftMotor;
  private VelocityVoltage leftLiftMotorVelocityVoltage, rightLiftMotorVelocityVoltage;
  private PositionVoltage leftLiftMotorPositionVoltage, rightLiftMotorPositionVoltage;
  private DigitalInput climberLiftZeroSensor;
  public Debouncer climberLiftDebouncer = new Debouncer(0.05);

  /* Creates a new ClimberLift. */
  public ClimberLift() {
    /*leftLiftMotor = new TalonFX(-1, kCANbus);
    rightLiftMotor = new TalonFX(-1, kCANbus);*/

    leftLiftMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
    rightLiftMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftLiftMotorPositionVoltage = new PositionVoltage(0).withSlot(0);
    rightLiftMotorPositionVoltage = new PositionVoltage(0).withSlot(0);

    climberLiftZeroSensor = new DigitalInput(0);
    //configureHardware();
  }

  @Override
  public void periodic(){
    // This method will be called once per scheduler run
    isAtZero = getIsAtZero();
    if (isAtZero){
      setZero();// if lift hits the bottom the position restes
    }
    SmartDashboard.putBoolean("isLiftAtZero", isAtZero);
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

  public void setZero(){
    position = 0.0;//TODO: make sure this actually works
    leftLiftMotor.setPosition(0);
    rightLiftMotor.setPosition(0);
  }

  public void setBrakeMode(Boolean mode){
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
  /*public void configureHardware(){

    var leftLiftMotorClosedLoopConfig = new SlotConfigs();
    leftLiftMotorClosedLoopConfig.withKP(leftKP);
    leftLiftMotorClosedLoopConfig.withKI(leftKI);
    leftLiftMotorClosedLoopConfig.withKD(leftKD);
    leftLiftMotorClosedLoopConfig.withKV(leftKV);

    var error = leftLiftMotor.getConfigurator().apply(leftLiftMotorClosedLoopConfig, 0.5);

    var rightLiftMotorClosedLoopConfig = new SlotConfigs();
    rightLiftMotorClosedLoopConfig.withKP(rightKP);
    rightLiftMotorClosedLoopConfig.withKI(rightKI);
    rightLiftMotorClosedLoopConfig.withKD(rightKD);
    rightLiftMotorClosedLoopConfig.withKV(rightKV);

    error = rightLiftMotor.getConfigurator().apply(rightLiftMotorClosedLoopConfig, 0.5);

    var leftLiftMotorConfig = new TalonFXConfiguration();//TODO: make sure config matches physical robot
    leftLiftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftLiftMotor.getConfigurator().apply(leftLiftMotorConfig);

    //var rightLiftMotorConfig = new TalonFXConfiguration();
    //rightLiftMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    //rightLiftMotor.getConfigurator().apply(rightLiftMotorConfig);

    leftLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
    //TODO: make sure to test ungeared setup before gearing
    rightLiftMotor.setControl(new Follower(leftLiftMotor.getDeviceID(), false));

    leftLiftMotor.setPosition(0);
    rightLiftMotor.setPosition(0);

    System.out.println("Climber Configured");
  }*/
}
