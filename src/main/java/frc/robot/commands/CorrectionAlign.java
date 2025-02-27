// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.List;

import org.opencv.photo.Photo;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CorrectionAlign extends Command {
  /** Creates a new creepAlign. */
  Drivetrain drivetrain;
  int tagID;
  AprilTagFinder finder;
  Transform2d offset;
  PhotonTrackedTarget fCTag = new PhotonTrackedTarget();

  PIDController xController;
  PIDController yController;
  PIDController thetaController;

  double xVelocity;
  double yVelocity;
  double wVelocity;

  double maxLinearVelocity = 1.0;
  double maxRotationVelocity = 1.0;

  // offset in tag coordinate system
  public CorrectionAlign(Drivetrain drivetrain, int tagID, AprilTagFinder finder, Transform2d offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.tagID = tagID;
    this.finder = finder;
    this.offset = offset;

    xVelocity = 0;
    yVelocity = 0;
    wVelocity = 0;

    xController = new PIDController(
      1.5, 
      0.0, 
      0.03
    );

    yController = new PIDController(
      1.5, 
      0.0, 
      0.03
    );

    thetaController = new PIDController(
      1.875, 
      0.0,
      0.03
    );

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset();
    yController.reset();
    thetaController.reset();
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Correction Align Execute");
    List<PhotonTrackedTarget> fCTags = finder.getFCCurrentTagData();

    for(PhotonTrackedTarget tag : fCTags) {
        if(tag.getFiducialId() == tagID) {
            fCTag = tag;
            break;
        }
    }

    if(fCTag.getFiducialId() != tagID) {
      for(int i = 0; i < 100; i++) {
        System.out.println("Don't have the desired tag");
      }
      return;
    }

    Transform3d robotToTag = finder.getRobotToFCCam().plus(fCTag.getBestCameraToTarget());
    Transform2d robotToTag2d = new Transform2d(robotToTag.getX(), robotToTag.getY(), robotToTag.getRotation().toRotation2d());
    Transform2d targetPose = offset.inverse();

    SmartDashboard.putNumber("Target Pose X", targetPose.getX());
    SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
    SmartDashboard.putNumber("Target Pose W", targetPose.getRotation().getRadians());

    
    // xVelocity = xController.calculate(targetPose.getX(), robotToTag2d.getX());
    // yVelocity = yController.calculate(targetPose.getY(), robotToTag2d.getY());
    // wVelocity = thetaController.calculate(targetPose.getRotation().getRadians(), robotToTag2d.getRotation().getRadians());

    // xVelocity = targetPose.getX() - robotToTag2d.getX();
    // yVelocity = targetPose.getY() - robotToTag2d.getY();
    // wVelocity = targetPose.getRotation().getRadians() - robotToTag2d.getRotation().getRadians();

    xVelocity = robotToTag2d.getX() - targetPose.getX();
    yVelocity = robotToTag2d.getY() - targetPose.getY();
    wVelocity = robotToTag2d.getRotation().getRadians() - targetPose.getRotation().getRadians();

    SmartDashboard.putNumber("xVelocity", xVelocity);
    SmartDashboard.putNumber("yVelocity", yVelocity);
    SmartDashboard.putNumber("wVelocity", wVelocity);

    xVelocity = MathUtil.clamp(xVelocity, -maxLinearVelocity, maxLinearVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxLinearVelocity, maxLinearVelocity);
    wVelocity = MathUtil.clamp(wVelocity, -maxRotationVelocity, maxRotationVelocity);

    ChassisSpeeds speed = new ChassisSpeeds(xVelocity, yVelocity, wVelocity);

    drivetrain.setTargetChassisSpeeds(
      ChassisSpeeds.fromRobotRelativeSpeeds(speed, drivetrain.getOdometry().getRotation())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
