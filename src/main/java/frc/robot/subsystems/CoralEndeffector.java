// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


// For the LaserCAN Sensor:
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Add your docs here. */
public class CoralEndeffector extends SubsystemBase {
    private final String kCANbus = "rio";
    private final double leftKP = 0.2;
    private final double leftKD = 0.0;
    private final double leftKI = 0.0;
    private final double leftKV = 0.12; // Kraken.

    private final double minCoralDistance = 0.03;

    private double velocity;
    private double position;
    private double load;
    private double commandedVelocity;
    private double coralDistance;
    private boolean hasCoral;

    private TalonFX motor;
    private VelocityVoltage motorVelocityVoltage;
   
    // LaserCAN Sensor:
    private LaserCan laserCAN;

    public CoralEndeffector() {
        hasCoral = false;
        motor = new TalonFX(21, kCANbus);
        
        motorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        // Sensor setup:
        laserCAN = new LaserCan(22);


        configureHardware();
    }

    @Override
    public void periodic() {

        // TODO: Scale factors.
        velocity = motor.getVelocity().refresh().getValueAsDouble();
        position = motor.getPosition().refresh().getValueAsDouble();

        load = motor.getTorqueCurrent().getValueAsDouble();
        
        // Read the coral sensor.
        LaserCan.Measurement measurement = laserCAN.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            coralDistance = measurement.distance_mm * 0.001; // mm's
            if (coralDistance < minCoralDistance) hasCoral = true;
            else hasCoral = false;
        } else {
            coralDistance = 999.0;
            hasCoral = false;
        }

        // Send motor command:
        motor.setControl(motorVelocityVoltage.withVelocity(commandedVelocity));

        SmartDashboard.putNumber("[CORAL End Effector] distance", coralDistance);
        SmartDashboard.putBoolean("[CORAL End Effector] has Coral", hasCoral);
        SmartDashboard.putNumber("[CORAL End Effector] velocity", velocity);
        SmartDashboard.putNumber("[CORAL End Effector] command", commandedVelocity);
        SmartDashboard.putNumber("[CORAL End Effector] load", load);
       
    }
    
    
    public double getPosition(){
        return position;
    }

    public void setVelocity(double velocity){
        // TODO: Scale factors.
        this.commandedVelocity = -velocity;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getLoad() {
        return load;
    }

    public boolean getHasCoral() {
        return hasCoral;
    }

    public void configureHardware() {

    var motorConfig = new TalonFXConfiguration(); //TODO check configs with robots
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motor.getConfigurator().apply(motorConfig);

    var motorClosedLoopConfig = new SlotConfigs();
    motorClosedLoopConfig.withKP(leftKP);
    motorClosedLoopConfig.withKI(leftKI);
    motorClosedLoopConfig.withKD(leftKD);
    motorClosedLoopConfig.withKV(leftKV);

    var error = motor.getConfigurator().apply(motorClosedLoopConfig, 0.5);

    motor.setNeutralMode(NeutralModeValue.Coast);//TODO consider changing brakemode (also test ungeared setup before gearing)

    motor.setPosition(0);

    // Laser CAN Setup:
    try {
        laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
        laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
        laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed for LaserCAN: " + e);
      }

    System.out.println("Coral Endeffector was configured");
  }
}
