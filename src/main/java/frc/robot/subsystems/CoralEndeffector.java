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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class CoralEndeffector extends SubsystemBase {
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

    private double velocity;
    private double position;
    private double load;
    private double rightLoad;
    private double leftLoad;

    private TalonFX leftEndeffectorMotor, rightEndeffectorMotor;
    private VelocityVoltage leftEndeffectorMotorVelocityVoltage, rightEndeffectorMotorVelocityVoltage;
    private PositionVoltage leftEndeffectorMotorPositionVoltage, rightEndeffectorMotorPositionVoltage;
    private DigitalInput EndeffectorLeftZeroSensor;
    public Debouncer coralEndeffectorDebouncer = new Debouncer(0.05);

    public CoralEndeffector(){

    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    public void setPosition(double position){
        this.position = position;
    }
    public double getPosition(){
        return position;
    }
    public void setVelocity(double velocity){
        this.velocity = velocity;
    }
    public double getVelocity(){
            return velocity;
    }
    public double getLoad(){
        return load;
    }
    public boolean getHasCoral(){
    //Pull from sensor
        return false;
    }

    public void configureHardware(){

    var leftEndeffectorMotorClosedLoopConfig = new SlotConfigs();
    leftEndeffectorMotorClosedLoopConfig.withKP(leftKP);
    leftEndeffectorMotorClosedLoopConfig.withKI(leftKI);
    leftEndeffectorMotorClosedLoopConfig.withKD(leftKD);
    leftEndeffectorMotorClosedLoopConfig.withKV(leftKV);

    var error = leftEndeffectorMotor.getConfigurator().apply(leftEndeffectorMotorClosedLoopConfig, 0.5);

    var rightEndeffectorMotorClosedLoopConfig = new SlotConfigs();
    rightEndeffectorMotorClosedLoopConfig.withKP(rightKP);
    rightEndeffectorMotorClosedLoopConfig.withKI(rightKI);
    rightEndeffectorMotorClosedLoopConfig.withKD(rightKD);
    rightEndeffectorMotorClosedLoopConfig.withKV(rightKV);

    error = rightEndeffectorMotor.getConfigurator().apply(rightEndeffectorMotorClosedLoopConfig, 0.5);

    var leftEndeffectorMotorConfig = new TalonFXConfiguration();//TODO check configs with robots
    leftEndeffectorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftEndeffectorMotor.getConfigurator().apply(leftEndeffectorMotorConfig);

    leftEndeffectorMotor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)
    rightEndeffectorMotor.setNeutralMode(NeutralModeValue.Coast);
    rightEndeffectorMotor.setControl(new Follower(leftEndeffectorMotor.getDeviceID(), true));

    leftEndeffectorMotor.setPosition(0);
    rightEndeffectorMotor.setPosition(0);

    System.out.println("yay the coral Endeffector was configured");
  }
}
