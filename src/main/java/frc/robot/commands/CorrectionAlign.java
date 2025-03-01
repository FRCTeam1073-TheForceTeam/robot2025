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


  Transform3d camToTag;
  Transform3d robotToTag;
  Transform2d robotToTag2d;
  Transform2d targetTransform;

  Pose2d cameraPose;
  Pose2d currentPose;
  Pose2d tagPos;
  Pose2d targetPose;

  double xVelocity;
  double yVelocity;
  double wVelocity;

  double maxLinearVelocity = 0.5;
  double maxRotationVelocity = 0.5;

  ChassisSpeeds speed;

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

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the tag and position
    List<PhotonTrackedTarget> fCTags = finder.getFCCurrentTagData();
    for(PhotonTrackedTarget tag : fCTags) {
      if(tag.getFiducialId() == tagID) {
          fCTag = tag;
          break;
      }
    }
    if(fCTag.getFiducialId() != tagID) {
      return;
    }

    camToTag = fCTag.getBestCameraToTarget();
    currentPose = drivetrain.getOdometry();
    robotToTag = finder.getRobotToFCCam().plus(camToTag);
    robotToTag2d = new Transform2d(robotToTag.getX(), robotToTag.getY(), robotToTag.getRotation().toRotation2d());
    tagPos = currentPose.plus(robotToTag2d); // for elastic
    
    targetTransform = robotToTag2d.plus(offset);
    targetPose = new Pose2d(currentPose.getX() + targetTransform.getX(), currentPose.getY() + targetTransform.getY(), currentPose.getRotation().plus(targetTransform.getRotation()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    xVelocity = targetTransform.getX();
    yVelocity = targetTransform.getY();
    wVelocity = targetTransform.getRotation().getRadians();

    xVelocity = MathUtil.clamp(xVelocity, -maxLinearVelocity, maxLinearVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maxLinearVelocity, maxLinearVelocity);
    wVelocity = MathUtil.clamp(wVelocity, -maxRotationVelocity, maxRotationVelocity);

    SmartDashboard.putNumber("Init Odo X", currentPose.getX());
    SmartDashboard.putNumber("Init Odo Y", currentPose.getY());
    SmartDashboard.putNumber("Init Odo W", currentPose.getRotation().getRadians());

    SmartDashboard.putNumber("Tag Position X", tagPos.getX());
    SmartDashboard.putNumber("Tag Position Y", tagPos.getY());
    SmartDashboard.putNumber("Tag Position W", tagPos.getRotation().getRadians());

    SmartDashboard.putNumber("target Transform X", targetTransform.getX());
    SmartDashboard.putNumber("target Transform Y", targetTransform.getY());
    SmartDashboard.putNumber("target Transform W", targetTransform.getRotation().getRadians());


    SmartDashboard.putNumber("Target Pose X", targetPose.getX());
    SmartDashboard.putNumber("Target Pose Y", targetPose.getY());
    SmartDashboard.putNumber("Target Pose W", targetPose.getRotation().getRadians());

    SmartDashboard.putNumber("xVelocity", xVelocity);
    SmartDashboard.putNumber("yVelocity", yVelocity);
    SmartDashboard.putNumber("wVelocity", wVelocity);

    speed = new ChassisSpeeds(xVelocity, yVelocity, wVelocity);

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
