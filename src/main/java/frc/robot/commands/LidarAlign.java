// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lidar;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LidarAlign extends Command {
  /** Creates a new LidarAlign. */
  Lidar lidar;
  boolean hasLine = true;
  Drivetrain drivetrain;
  double lidarSlope;
  double angleToRotate;
  double xToDrive;
  double thetaVelocity;
  double vx;
  int sign = 0;
  PIDController thetaController;
  PIDController xController;
  
  public LidarAlign(Lidar lidar, Drivetrain drivetrain) {
    this.lidar = lidar;
    this.drivetrain = drivetrain;
    thetaController = new PIDController(0.8, 0, 0.01);
    xController = new PIDController(0.8, 0, 0.01);
    thetaController.enableContinuousInput(-Math.PI/2, Math.PI/2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    xController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // TODO: I would not duplicate exit conditions, etc. here. Just compute a response here and stop/exit using isFinished()
    // The duplication makes it fragile to change the tunings.
    if(lidar.getAvgX() < 1.5 && Math.abs(lidar.getSlope()) < 5){
      angleToRotate = -Math.atan(lidar.getSlope());
      if(lidar.getAvgX() > 0.43){
        //TODO: check if setpoint is correct
        vx = xController.calculate(lidar.getAvgX(), 0.4);
        // TODO: I would use a lower clamp min to prevent oscillation. Just let it run to 0.05
        // TODO: I would ue a lower maximum as well... this is alignment not speed racing... maybe 0.7 tops.
        vx = MathUtil.clamp(vx, 0.2, 2);
        // TODO: Check which heading source we're using here. getOdometryThetaRadians()?
        // 
      thetaVelocity = thetaController.calculate(drivetrain.getGyroHeadingRadians(), drivetrain.getGyroHeadingRadians() + angleToRotate);
      if(thetaVelocity < 0){
        // TODO: Clamp to much smaller extreme value 2 radians/second is crazy fast maybe 0.7 tops.
        // TODO: Use a lower min clamp -0.03? to prevent oscillation.
        thetaVelocity = MathUtil.clamp(thetaVelocity, -2, -0.2);
      }
      else if(thetaVelocity > 0){
        // TODO: Clamp to much smaller extreme value... see above.
        // TODO: Use a lower min clamp ... see above. low maybe 0.03
        thetaVelocity = MathUtil.clamp(thetaVelocity, 0.2, 2);
      }
      drivetrain.setTargetChassisSpeeds(new ChassisSpeeds(vx, 0, thetaVelocity));
      }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setTargetChassisSpeeds( new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (lidar.getAvgX() <= 0.42 && Math.abs(lidar.getSlope()) <= 0.05){ //TODO change values or add vaiable in smartdashboard
      return true;
    }
    if (lidar.getAvgX() <= 0.42){
      return true;
    }
    return false;
  }
}