// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberLift extends SubsystemBase {
  private final String kCANbus = "CANivore";

  private double load;
  private double velocity;
  private double position;
  private boolean brakeMode;

  //TODO: Name Motors Appropriately
  private TalonFX motor1, motor2;
  private VelocityVoltage motor1VelocityVoltage, motor2VelocityVoltage;
  private PositionVoltage motor1PositionVoltage, motor2PositionVoltage;

  /* Creates a new ClimberLift. */
  public ClimberLift() {
    //TODO: Name Motors Appropriately
    /*motor1 = new TalonFX(-1, kCANbus);
    motor2 = new TalonFX(-1, kCANbus);*/

    motor1VelocityVoltage = new VelocityVoltage(0).withSlot(0);
    motor2VelocityVoltage = new VelocityVoltage(0).withSlot(0);

    motor1PositionVoltage = new PositionVoltage(0).withSlot(0);
    motor2PositionVoltage = new PositionVoltage(0).withSlot(0);

    //configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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

  public void setZero(){
    position = 0.0;
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
  /*public void configureHardware(){
    //TODO: Name Motors Appropriately
    var motor1ClosedLoopConfig = new SlotConfigs();
    motor1ClosedLoopConfig.withKP(0.1);
    motor1ClosedLoopConfig.withKI(0.0);
    motor1ClosedLoopConfig.withKD(0.0);
    motor1ClosedLoopConfig.withKV(0.0);

    var error = motor1.getConfigurator().apply(motor1ClosedLoopConfig, 0.5);

    //TODO: Name Motors Appropriately
    var motor2ClosedLoopConfig = new SlotConfigs();
    motor2ClosedLoopConfig.withKP(0.1);
    motor2ClosedLoopConfig.withKI(0.0);
    motor2ClosedLoopConfig.withKD(0.0);
    motor2ClosedLoopConfig.withKV(0.0);

    error = motor2.getConfigurator().apply(motor2ClosedLoopConfig, 0.5);

    motor1.setNeutralMode(NeutralModeValue.Brake);
    motor2.setNeutralMode(NeutralModeValue.Brake);

    //TODO: Name Motors Appropriately
    motor1.setPosition(0);
    motor2.setPosition(0);

    System.out.println("Climber Configured");
  }*/
}
