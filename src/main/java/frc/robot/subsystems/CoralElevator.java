// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralElevator extends SubsystemBase {
  private final String kCANbus = "rio";
  private final double leftKP = 0.15;
  private final double leftKD = 0.02;
  private final double leftKI = 0.0;
  private final double leftKV = 0.12; // Kraken kV value.

  private final double maxLoad = 40.0; // TODO: Tune max load.
  private final double maxPosition = 5.0; // TODO: Set to maximum position.


  private double position;
  private double velocity;
  private double leftLoad;
  private double rightLoad;
  private double load;
  private double commandedVelocity;
  private boolean brakemode;
  private boolean isAtZero;

  private TalonFX leftElevatorMotor, rightElevatorMotor;
  private VelocityVoltage leftElevatorMotorVelocityVoltage;
  private DigitalInput leftZeroSensor;
  private DigitalInput rightZeroSensor;
  public Debouncer zeroDebouncer = new Debouncer(0.05);


  public CoralElevator() {
    brakemode = false;
    leftElevatorMotor = new TalonFX(19, kCANbus);
    rightElevatorMotor = new TalonFX(20, kCANbus);

    leftElevatorMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    leftZeroSensor = new DigitalInput(2); // TODO: Change channel.
    rightZeroSensor = new DigitalInput(3);// TODO: Change channel.

    configureHardware();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // TODO: Need to use scale factors from ratio, etc. units need to be meters.
    velocity = leftElevatorMotor.getVelocity().refresh().getValueAsDouble();
    position = leftElevatorMotor.getPosition().refresh().getValueAsDouble();

    leftLoad = leftElevatorMotor.getTorqueCurrent().getValueAsDouble();
    rightLoad = rightElevatorMotor.getTorqueCurrent().getValueAsDouble();

    load = Math.max(leftLoad, rightLoad);
    boolean hitHardStop = (velocity < 0.0) && (load > maxLoad); // MOving down, peak load => reset.
    isAtZero = zeroDebouncer.calculate(leftZeroSensor.get() | rightZeroSensor.get() | hitHardStop); // Compute debounced logical or.

    if (isAtZero){
      setZero();
      if (commandedVelocity < 0.0) commandedVelocity = 0.0; // Velocity hard-limit at bottom of travel can only go up from here.
    }

    if (position > maxPosition && commandedVelocity > 0.0) commandedVelocity = 0.0; // Don't go past maximum height.


    leftElevatorMotor.setControl(leftElevatorMotorVelocityVoltage.withVelocity(velocity));

    SmartDashboard.putBoolean("[CORAL ELEVATOR] at zero", isAtZero);
    SmartDashboard.putBoolean("[CORAL ELEVATOR] brake mode", brakemode);
    SmartDashboard.putNumber("[CORAL ELEVATOR] position", position);
    SmartDashboard.putNumber("[CORAL ELEVATOR] velocity", velocity);
    SmartDashboard.putNumber("[CORAL ELEVATOR] command", commandedVelocity);
  }

  public double getPosition(){// where motor is
    return position;
  }

  public void setZero(){
    position = 0.0;
    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);
  }

  public void setVelocity(double velocity) {
    // TODO: Convert command internall from meters/second to internal commandedVelocity value using ratio, etc.
    this.commandedVelocity = velocity;
  }

  public boolean isCoralElevatorAtBottom(){
    return isAtZero;
  }


  public double getMaxLoad(){
    return maxLoad;
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

  public void configureHardware() {

    var leftElevatorMotorConfig = new TalonFXConfiguration();//TODO check configs with robots
    leftElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftElevatorMotor.getConfigurator().apply(leftElevatorMotorConfig);
    rightElevatorMotor.getConfigurator().apply(leftElevatorMotorConfig); // Same config as other motor to start.


    var leftElevatorMotorClosedLoopConfig = new SlotConfigs();
    leftElevatorMotorClosedLoopConfig.withKP(leftKP);
    leftElevatorMotorClosedLoopConfig.withKI(leftKI);
    leftElevatorMotorClosedLoopConfig.withKD(leftKD);
    leftElevatorMotorClosedLoopConfig.withKV(leftKV);

    var error = leftElevatorMotor.getConfigurator().apply(leftElevatorMotorClosedLoopConfig, 0.5);
    // TODO hardware error checking.

    //TODO consider changing brakemode (also test ungeared setup before gearing)
    leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);

    leftElevatorMotor.setPosition(0);
    rightElevatorMotor.setPosition(0);

    rightElevatorMotor.setControl(new Follower(leftElevatorMotor.getDeviceID(), true));


    System.out.println("Coral Elevator configured");
  }
}