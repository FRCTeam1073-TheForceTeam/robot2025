// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;


/** Add your docs here. */
public class SwerveModuleConfig 
{
    public int moduleNumber = -1;
    public double gearRatio = 6.75;
    public double wheelDiameterMeters = 0.1016;
    public Translation2d position = new Translation2d(0,0);
    //public double tickPerMeter = 1000;
    public double rotationsPerMeter = gearRatio / (wheelDiameterMeters * Math.PI);
    public double radiansPerRotation = (150.0 / 7.0) / (Math.PI * 2);
    //public double tickPerRadian = 1000;
    //public double steerRotationOffset = 0;
    public double steerCurrentLimit = 20;
    public double driveCurrentLimit = 35;
    public double steerCurrentThreshold = 12;
    public double driveCurrentThreshold = 22;
    public double steerCurrentThresholdTime = 0.1;
    public double driveCurrentThresholdTime = 0.25;
    public double steerP = 0;
    public double steerI = 0;
    public double steerD = 0;
    public double steerF = 0;
    public double driveP = 0;
    public double driveI = 0;
    public double driveD = 0;
    public double driveF = 0;
    public double driveMaxIntegrator = 400.0;
    public double steerMaxIntegrator = 400.0;    


    /**SwerveModuleConfig contstructor sets PIDF values and current limits
     * 
     */
    public SwerveModuleConfig()
    {
        driveP = Preferences.getDouble("Drive.Drive.Kp", 0.3); 
        driveI = Preferences.getDouble("Drive.Drive.Ki", 0.0);
        driveD = Preferences.getDouble("Drive.Drive.Kd", 0.001);
        driveF = Preferences.getDouble("Drive.Drive.Kf", 0.115);
        driveMaxIntegrator = Preferences.getDouble("Drive.Drive.MaxIntegrator", 400.0);
        driveCurrentLimit = Preferences.getDouble("Drive.Drive.CurrentLimit", 30);
        driveCurrentThreshold = Preferences.getDouble("Drive.Drive.CurrentThreshold", 35);


        //tickPerRadian = Preferences.getDouble("Drive.Steer.TicksPerRadian", 4096.0/(2*Math.PI)); // 4,096 ticks per rotation, converted to radians
        steerP = Preferences.getDouble("Drive.Steer.Kp", 30.0); 
        steerI = Preferences.getDouble("Drive.Steer.Ki", 4.0);
        steerD = Preferences.getDouble("Drive.Steer.Kd", 1.0);
        steerF = Preferences.getDouble("Drive.Steer.Kf", 0.0);
        steerMaxIntegrator = Preferences.getDouble("Drive.Steer.MaxIntegrator", 400.0);
        steerCurrentLimit = Preferences.getDouble("Drive.Steer.CurrentLimit", 12);
        steerCurrentThreshold = Preferences.getDouble("Drive.Steer.CurrentThreshold", 15);
    }

    //Initializes preferences for PIDF values for both drive and steer motors, current limits and current threshold
    public static void initPreferences() {
        Preferences.initDouble("Drive.Drive.Kp", 0.3);
        Preferences.initDouble("Drive.Drive.Ki", 0.0);
        Preferences.initDouble("Drive.Drive.Kd", 0.001);
        Preferences.initDouble("Drive.Drive.Kf", 0.115);
        Preferences.initDouble("Drive.Drive.MaxIntegrator", 400.0);
        Preferences.initDouble("Drive.Drive.CurrentLimit", 30);
        Preferences.initDouble("Drive.Drive.CurrentThreshold", 35);
    
        Preferences.initDouble("Drive.Steer.Kp", 30.0);
        Preferences.initDouble("Drive.Steer.Ki", 4.0);
        Preferences.initDouble("Drive.Steer.Kd", 1.0);
        Preferences.initDouble("Drive.Steer.Kf", 0.0);
        Preferences.initDouble("Drive.Steer.MaxIntegrator", 400.0);
        Preferences.initDouble("Drive.Steer.CurrentLimit", 12);
        Preferences.initDouble("Drive.Steer.CurrentThreshold", 15);
    }
}