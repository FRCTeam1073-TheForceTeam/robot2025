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
import edu.wpi.first.epilogue.logging.LazyBackend;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CANdleControl;


/** Add your docs here. */
public class CoralEndeffector extends SubsystemBase {
    private final String kCANbus = "rio";
    private final double leftKP = 0.2;
    private final double leftKD = 0.0;
    private final double leftKI = 0.0;
    private final double leftKV = 0.12; // Kraken.

    private final double minCoralDistance = 0.03;
    private final double minReefDistance = 0.45;

    private double velocity;
    private double position;
    private double load;
    private double commandedVelocity;
    private double coralDistance;
    private boolean hasCoral = false;

    private double reefDistance;
    private boolean hasReef = false;

    private TalonFX motor;
    private VelocityVoltage motorVelocityVoltage;
    private CANdleControl candle;
   
    // LaserCAN Sensor:
    private LaserCan laserCANCoral;
    private LaserCan laserCANReef;

    public CoralEndeffector(CANdleControl candleControl) {
        hasCoral = false;
        motor = new TalonFX(21, kCANbus);
        
        motorVelocityVoltage = new VelocityVoltage(0).withSlot(0);

        // Sensor setup:
        laserCANCoral = new LaserCan(22);
        laserCANReef = new LaserCan(24);
        candle = candleControl;

        configureHardware();
    }

    @Override
    public void periodic() {

        // TODO: Scale factors.
        velocity = motor.getVelocity().getValueAsDouble();
        position = motor.getPosition().getValueAsDouble();

        load = motor.getTorqueCurrent().getValueAsDouble();
        
        // Read the coral sensor.
        LaserCan.Measurement coral_measurement = laserCANCoral.getMeasurement();
        if (coral_measurement != null && coral_measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            coralDistance = coral_measurement.distance_mm * 0.001; // mm's
            if (coralDistance < minCoralDistance) hasCoral = true;
            else hasCoral = false;
        } else {
            coralDistance = 999.0;
            hasCoral = false;
        }

        // Read the reef sensor.
        LaserCan.Measurement reef_measurement = laserCANReef.getMeasurement();
        if (reef_measurement != null && reef_measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            reefDistance = reef_measurement.distance_mm * 0.001; // mm's
            if (reefDistance < minReefDistance){
                hasReef = true;
            }
            else {
                hasReef = false;
            }
        } else {
            reefDistance = 999.0;
            hasReef = false;
        }

        if (hasReef){
            candle.setRGB(0, 255, 0, 0, 8); //set candle to green
        }

        else{
            candle.setRGB(255, 0, 0, 0, 8); //set candle to red
        }
        // Send motor command:
        motor.setControl(motorVelocityVoltage.withVelocity(commandedVelocity));

        SmartDashboard.putNumber("Coral End Effector/Coral Distance", coralDistance);
        SmartDashboard.putBoolean("Coral End Effector/Has Coral", hasCoral);
        SmartDashboard.putNumber("Coral End Effector/ velocity", velocity);
        SmartDashboard.putNumber("Coral End Effector/command", commandedVelocity);
        SmartDashboard.putNumber("Coral End Effector/load", load);
        SmartDashboard.putNumber("Coral End Effector/Reef Distance", reefDistance);
        SmartDashboard.putBoolean("Coral End Effector/ Has Reef", hasReef);
    
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

    public double getCoralDistance() {
        return coralDistance;
    }

    public boolean getHasCoral() {
        return hasCoral;
    }

    public double getReefDistance() {
        return reefDistance;
    }

    public boolean getHasReef() {
        return hasReef;
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
        laserCANCoral.setRangingMode(LaserCan.RangingMode.SHORT);
        laserCANCoral.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 8, 8));
        laserCANCoral.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
      } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed for LaserCAN: " + e);
      }

    System.out.println("Coral Endeffector was configured");
  }
}
