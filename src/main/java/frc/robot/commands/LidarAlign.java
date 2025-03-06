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
    if(lidar.getAvgX() < 1.5 && Math.abs(lidar.getSlope()) < 5){
      angleToRotate = -Math.atan(lidar.getSlope());
      if(xToDrive > 0){
        //TODO: check if setpoint is correct
        vx = xController.calculate(lidar.getAvgX(), 0.4);
        vx = MathUtil.clamp(vx, 0.2, 2);
      thetaVelocity = thetaController.calculate(drivetrain.getGyroHeadingRadians(), drivetrain.getGyroHeadingRadians() + angleToRotate);
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
    if (lidar.getAvgX() <= 0.43 && Math.abs(angleToRotate) <= 0.01){ //TODO change values or add vaiable in smartdashboard
      return true;
    }
    if (lidar.getAvgX() <= 0.43){
      return true;
    }
    return false;
  }
}