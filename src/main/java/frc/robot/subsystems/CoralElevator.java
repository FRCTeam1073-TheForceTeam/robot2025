// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
  private final double backKP = 0.15;
  private final double backKD = 0.02;
  private final double backKI = 0.0;
  private final double backKV = 0.12; // Kraken kV value.

  private final double maxLoad = 40.0; // TODO: Tune max load.
  private final double maxPosition = 5.0; // TODO: Set to maximum position.


  private double position;
  private double velocity;
  private double backLoad;
  private double frontLoad;
  private double load;
  private double commandedVelocity;
  private boolean brakemode;
  private boolean isAtZero;

  private TalonFX backElevatorMotor, frontElevatorMotor;
  private VelocityVoltage frontElevatorMotorVelocityVoltage;
  private DigitalInput zeroSensor;
  public Debouncer zeroDebouncer = new Debouncer(0.05);


  public CoralElevator() {
    brakemode = false;
    backElevatorMotor = new TalonFX(19, kCANbus);
    frontElevatorMotor = new TalonFX(20, kCANbus);

    frontElevatorMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

    zeroSensor = new DigitalInput(4);// TODO: Change channel.

    configureHardware();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // TODO: Need to use scale factors from ratio, etc. units need to be meters.
    velocity = backElevatorMotor.getVelocity().refresh().getValueAsDouble();
    position = backElevatorMotor.getPosition().refresh().getValueAsDouble();

    backLoad = backElevatorMotor.getTorqueCurrent().getValueAsDouble();
    frontLoad = frontElevatorMotor.getTorqueCurrent().getValueAsDouble();

    load = Math.max(backLoad, frontLoad);
    boolean hitHardStop = (velocity < 0.0) && (load > maxLoad); // MOving down, peak load => reset.
    isAtZero = zeroDebouncer.calculate(zeroSensor.get() | hitHardStop); // Compute debounced logical or.

    if (isAtZero){
      setZero();
      if (commandedVelocity < 0.0) commandedVelocity = 0.0; // Velocity hard-limit at bottom of travel can only go up from here.
    }

    if (position > maxPosition && commandedVelocity > 0.0) commandedVelocity = 0.0; // Don't go past maximum height.


    frontElevatorMotor.setControl(frontElevatorMotorVelocityVoltage.withVelocity(velocity));

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
    backElevatorMotor.setPosition(0);
    frontElevatorMotor.setPosition(0);
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
      backElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
      frontElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
    }
    else{
      backElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
      frontElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    brakemode = mode;
  }

  public void configureHardware() {

    var frontElevatorMotorConfig = new TalonFXConfiguration();//TODO check configs with robots
    frontElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    frontElevatorMotor.getConfigurator().apply(frontElevatorMotorConfig);
    backElevatorMotor.getConfigurator().apply(frontElevatorMotorConfig);
     // Same config as other motor to start.


    var frontElevatorMotorClosedLoopConfig = new SlotConfigs();
    frontElevatorMotorClosedLoopConfig.withKP(backKP);
    frontElevatorMotorClosedLoopConfig.withKI(backKI);
    frontElevatorMotorClosedLoopConfig.withKD(backKD);
    frontElevatorMotorClosedLoopConfig.withKV(backKV);

    var error = frontElevatorMotor.getConfigurator().apply(frontElevatorMotorClosedLoopConfig, 0.5);
    // TODO hardware error checking.

    //TODO consider changing brakemode (also test ungeared setup before gearing)
    backElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
    frontElevatorMotor.setNeutralMode(NeutralModeValue.Coast);

    CurrentLimitsConfigs frontElevatorCurrentLimitsConfigs = new CurrentLimitsConfigs();
    frontElevatorCurrentLimitsConfigs.withSupplyCurrentLimitEnable(true)
                            .withSupplyCurrentLimit(15)
                            .withSupplyCurrentLowerTime(0.25);

    frontElevatorMotor.getConfigurator().apply(frontElevatorCurrentLimitsConfigs);
    backElevatorMotor.getConfigurator().apply(frontElevatorCurrentLimitsConfigs);

    backElevatorMotor.setControl(new Follower(frontElevatorMotor.getDeviceID(), true));

    backElevatorMotor.setPosition(0);
    frontElevatorMotor.setPosition(0);

    System.out.println("Coral Elevator configured");
  }
}