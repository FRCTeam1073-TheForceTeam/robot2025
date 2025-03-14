// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.AprilTagFinder;

public class AlignToTagRelative extends Command 
{
  Drivetrain drivetrain;
  AprilTagFinder finder;
  int aprilTagID;
  OI oi;
  Localizer localizer;
  FieldMap fieldMap;
  MapDisplay mapDisplay;
  int slot = 0;

  Transform2d lastLocation; // Last location we've seen the tag in robot coordinates.
  Transform2d offset;
  Pose2d currentPose;
  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double xVelocity = 0.0;
  double yVelocity = 0.0;
  double wVelocity = 0.0;
  double xError = 0.0;
  double yError = 0.0;
  double wError = 0.0;
  int missCounter = 0;
  ChassisSpeeds speeds;


  private final static double maximumLinearVelocity = 1.0;   // Meters/second
  private final static double maximumRotationVelocity = 1.0; // Radians/second

  /** Creates a new alignToTag. */
  public AlignToTagRelative(Drivetrain drivetrain, AprilTagFinder finder, Localizer localizer, FieldMap fieldMap, MapDisplay mapDisplay, OI oi) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.finder = finder;
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.fieldMap = fieldMap;
    this.mapDisplay = mapDisplay;
    this.oi = oi;
    this.aprilTagID = -1;

    speeds = new ChassisSpeeds();

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
  public void initialize() 
  {
    double yOffset = 0.165;
    double endEffectorOffset = 0.1905;

    xController.reset();
    yController.reset();
    thetaController.reset();
    missCounter = 1; // Need to see it to start...

    currentPose = localizer.getPose();

    // temporary for testing the align
    // if (oi.getDriverXButton())
    // {
    //   slot = -1;
    //   offset = new Transform2d(0.45, -yOffset + endEffectorOffset, new Rotation2d(Math.PI));
    // }
    // else if (oi.getDriverAButton())
    // {
    //   slot = 0;
    //   offset = new Transform2d(0.45, endEffectorOffset, new Rotation2d(Math.PI));
    // }
    // else if (oi.getDriverYButton())
    // {
    //   slot = 1;
    //   offset = new Transform2d(0.45, yOffset + endEffectorOffset, new Rotation2d(Math.PI));
    // }

    slot = 0; //this is just a hack for testing
    offset = new Transform2d(0.45, endEffectorOffset, new Rotation2d(Math.PI));

    if (aprilTagID == -1)
    {
      if (slot != 2)
      {
        aprilTagID = fieldMap.getBestReefTagID(currentPose);
      }
    }
    SmartDashboard.putString("AlignTag", mapDisplay.aprilTagAssignments(aprilTagID));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // Drive on vision measurements.
    // Look for the tag in the currently tracked set:
    var tags = finder.getAllMeasurements();
    

    // Scan for the tag we're aligning with:
    boolean found = false;
    if(tags != null) {
      for (var tag : tags) {
        if (tag.tagID == aprilTagID) {
  
          /// Update last location of tag in robot coordinates with the offset...
          lastLocation = tag.relativePose.plus(offset).inverse();
          xError = Math.abs(lastLocation.getX());
          yError = Math.abs(lastLocation.getX());
          wError = Math.abs(lastLocation.getRotation().getRadians());
  
          /// We didn't miss it, we have it.
          if (missCounter > 0) missCounter--;
          found = true;
        }
      }
      if (!found) missCounter++; // We missed seeing it this time, we're using old location?
    }
    else {
      missCounter++;
    }

    if(lastLocation == null) {
      return;
    }

    // Within ~1/10th second.
    if (missCounter < 12) {
      // We have the tag, align to the offset robot relative position (make the robot center 0,0,0 from the location)
      xVelocity = xController.calculate(lastLocation.getX(), 0);
      yVelocity = yController.calculate(lastLocation.getY(), 0);
      wVelocity = thetaController.calculate(lastLocation.getRotation().getRadians(), 0);

      xVelocity = MathUtil.clamp(xVelocity, -maximumLinearVelocity, maximumLinearVelocity);
      yVelocity = MathUtil.clamp(yVelocity, -maximumLinearVelocity, maximumLinearVelocity);
      wVelocity = MathUtil.clamp(wVelocity, -maximumRotationVelocity, maximumRotationVelocity);
      speeds.vxMetersPerSecond = xVelocity;
      speeds.vyMetersPerSecond = yVelocity;
      speeds.omegaRadiansPerSecond = wVelocity;

    
    } else {
      // Missed target a lot... stop?
      speeds.vxMetersPerSecond = 0.0;
      speeds.vyMetersPerSecond = 0.0;
      speeds.omegaRadiansPerSecond = 0.0;
    }

    drivetrain.setTargetChassisSpeeds(speeds);

    SmartDashboard.putNumber("AlignToTag/TagId", aprilTagID);
    SmartDashboard.putNumber("AlignToTag/MissCounter", missCounter);
    SmartDashboard.putNumber("AlignToTag/X", lastLocation.getX());
    SmartDashboard.putNumber("AlignToTag/Y", lastLocation.getY());
    SmartDashboard.putNumber("AlignToTag/Omega", lastLocation.getRotation().getRadians());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    speeds.vxMetersPerSecond = 0.0;
    speeds.vyMetersPerSecond = 0.0;
    speeds.omegaRadiansPerSecond = 0.0;
    aprilTagID = -1;

    drivetrain.setTargetChassisSpeeds(speeds);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // We lost it for too long.
    if (missCounter >= 12) return true;

    // We're basically there.
    if (xError < 0.05 && yError < 0.03 && wError < 0.01)
      return true;
    else
      return false;
  }
}
