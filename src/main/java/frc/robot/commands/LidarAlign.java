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
    //covxyAtZero based on sqrt of covxy, isbad - if the varx is above 0.005 & varx/covxy > 0.04
    // if(lidar.getAngleToRotate() != Math.PI){
    //   thetaVelocity = thetaController.calculate(drivetrain.getOdometryThetaRadians(), lidar.getAngleToRotate());  
    //   thetaVelocity = MathUtil.clamp(thetaVelocity, 0.2, 1.5);
    //   if(thetaVelocity < 0){
    //       thetaVelocity = MathUtil.clamp(thetaVelocity, -2, -0.2);
    //   }
    //   else if(thetaVelocity > 0){
    //     thetaVelocity = MathUtil.clamp(thetaVelocity, 0.2, 2);
    //   }
    //   else if(thetaVelocity == 0){
    //     thetaVelocity = 0;
    //   }
   // }
    
      // xToDrive = lidar.getAverageX() - 0.4;
      // if(xToDrive > 0 && lidar.getAverageX() != 1){
      //   //TODO: check if setpoint is correct
      //   vx = xController.calculate(lidar.getAverageX(), 0.4);
      //   if(vx < 0){
      //     vx = MathUtil.clamp(vx, -2, -0.2);
      //   }
      //   if(vx > 0){
      //     vx = MathUtil.clamp(vx, 0.2, 2);
      //   }
      //   drivetrain.setTargetChassisSpeeds(
      //     new ChassisSpeeds(
      //      vx, 
      //      0, 
      //      thetaVelocity));
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setTargetChassisSpeeds( new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //TODO: change numbers
    // if(lidar.getAverageX() <= 0.43 && Math.abs(lidar.getAverageSlope()) < 0.01){
    //   return true;
    // }
    // if(lidar.getAverageX() <= 0.43){
    //   return true;
    // }
    //   return false;
    return true;
  }
}