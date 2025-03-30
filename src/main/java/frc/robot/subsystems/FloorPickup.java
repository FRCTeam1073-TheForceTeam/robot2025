// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class FloorPickup extends SubsystemBase{
    private final String kCANbus = "rio";
    private final double rollerkP = 0.2;
    private final double rollerkD = 0.0;
    private final double rollerkI = 0.01;
    private final double rollerkV = 0.12; // Kraken kV value.
    //private final double rollerkA = 0.01;
    //private final double rollerkS = 0.0;

    private final double pivotkP = 0.2;
    private final double pivotkD = 0.0;
    private final double pivotkI = 0.01;
    private final double pivotkV = 0.12; // Kraken kV value.
    //private final double pivotkA = 0.01;
    //private final double pivotkS = 0.0;

    private double rollerVelocity = 0.0;
    private double rollerLoad = 0.0;
    private double pivotPosition = 0.0;
    private double pivotVelocity = 0.0;
    private double pivotLoad = 0.0;

    private double pivotCommandedVelocity = 0.0;
    private double rollerCommandedVelocity = 0.0;
    private double pivotCommandedPosition = 0.0;
    private final double rollerMaxLoad = 0.0;
    private final double pivotMaxLoad = 0.0;
    private final double pivotMaxPosition = 0.0;
    public final double pivotMinPosition = 0.0;

    private TalonFX rollerMotor, pivotMotor;
    private VelocityVoltage rollerMotorVelocityVoltage, pivotMotorVelocityVoltage;
    private MotionMagicVoltage pivotPositionController;

    public FloorPickup(){
        rollerMotor = new TalonFX(27, kCANbus); //TODO fix can value
        pivotMotor = new TalonFX(28, kCANbus);   //TODO fix can vlaue

        rollerMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);
        pivotMotorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        pivotPositionController = new MotionMagicVoltage(0).withSlot(1);

        configureHardware();
    }

    @Override
    public void periodic() {
        rollerVelocity = rollerMotor.getVelocity().getValueAsDouble();
        rollerLoad = rollerMotor.getTorqueCurrent().getValueAsDouble();

        pivotVelocity = pivotMotor.getVelocity().getValueAsDouble();
        pivotPosition = pivotMotor.getPosition().getValueAsDouble();
        pivotLoad = pivotMotor.getTorqueCurrent().getValueAsDouble();

        pivotMotor.setControl(pivotMotorVelocityVoltage.withVelocity(pivotCommandedVelocity));
        rollerMotor.setControl(rollerMotorVelocityVoltage.withVelocity(rollerCommandedVelocity));

        SmartDashboard.putNumber("FloorPickup/Roller Commanded Velocity", rollerCommandedVelocity);
        SmartDashboard.putNumber("FloorPickup/Pivot Commanded Velocity", pivotCommandedVelocity);
        SmartDashboard.putNumber("FloorPickup/Roller Velocity", rollerVelocity);
        SmartDashboard.putNumber("FloorPickup/Pivot Velocity", pivotVelocity);
        SmartDashboard.putNumber("FloorPickup/Pivot Position", pivotPosition);
        SmartDashboard.putNumber("FloorPickup/Pivot Load", pivotLoad);
        SmartDashboard.putNumber("FloorPickup/Roller Load", rollerLoad);
    }

    public void setPivotPosition(double position){
        pivotCommandedPosition = position;
    }

    public void setPivotVelocity(double velocity){
        pivotCommandedVelocity = velocity;
    }

    public void setRollerVelocity(double velocity){
        rollerCommandedVelocity = velocity;
    }

    public double getPivotPosition(){
        return pivotPosition;
    }

    public double getPivotVelocity(){
        return pivotVelocity;
    }

    public double getPivotLoad(){
        return pivotLoad;
    }

    public double getRollerVelocity(){
        return rollerVelocity;
    }

    public double getRollerLoad(){
        return rollerLoad;
    }

    public void configureHardware() {

        var rollerMotorConfig = new TalonFXConfiguration(); //TODO check configs with robots
        var pivotMotorConfig = new TalonFXConfiguration(); //TODO check configs with robots
        rollerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        

        var mmConfigs = pivotMotorConfig.MotionMagic;
        mmConfigs.MotionMagicCruiseVelocity = 75; //TODO change mm numbers
        mmConfigs.MotionMagicAcceleration = 100;
        mmConfigs.MotionMagicJerk = 800;

        var rollerMotorClosedLoopConfig = new SlotConfigs();
        rollerMotorClosedLoopConfig.withKP(rollerkP);
        rollerMotorClosedLoopConfig.withKI(rollerkI);
        rollerMotorClosedLoopConfig.withKD(rollerkD);
        rollerMotorClosedLoopConfig.withKV(rollerkV);
        //rollerMotorClosedLoopConfig.withKS(rollerkS);

        var pivotMotorClosedLoopConfig = new SlotConfigs();
        pivotMotorClosedLoopConfig.withKP(pivotkP);
        pivotMotorClosedLoopConfig.withKI(pivotkI);
        pivotMotorClosedLoopConfig.withKD(pivotkD);
        pivotMotorClosedLoopConfig.withKV(pivotkV);
        //pivotElevatorMotorClosedLoopConfig.withKS(pivotkS);

        var rollerError = rollerMotor.getConfigurator().apply(rollerMotorClosedLoopConfig, 0.5);
        var pivotError = pivotMotor.getConfigurator().apply(pivotMotorClosedLoopConfig, 0.5);
        
        rollerMotor.getConfigurator().apply(rollerMotorConfig, 0.5);
        pivotMotor.getConfigurator().apply(pivotMotorConfig, 0.5);

        rollerMotor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)


        rollerMotor.setPosition(0);
        pivotMotor.setPosition(0);

        System.out.println("Floor Pickup was configured");
    }
}
