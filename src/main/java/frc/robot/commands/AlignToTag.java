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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.FieldMap;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTag extends Command 
{
  Drivetrain drivetrain;
  Localizer localizer;
  FieldMap fieldMap;
  MapDisplay mapDisplay;
  OI oi;
  int aprilTagID;
  Pose2d targetPose;
  PIDController xController;
  PIDController yController;
  PIDController thetaController;
  double xVelocity;
  double yVelocity;
  double wVelocity;
  double xError;
  double yError;
  double wError;
  int slot;
  boolean isRed;
  boolean terminate = false;

  private final static double maximumLinearVelocity = 3.0;   // Meters/second
  private final static double maximumRotationVelocity = 3.0; // Radians/second

  /** Creates a new alignToTag. */
  public AlignToTag(Drivetrain drivetrain, Localizer localizer, FieldMap fieldMap, MapDisplay mapDisplay, OI oi, boolean terminate, int slot) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.localizer = localizer;
    this.fieldMap = fieldMap;
    this.mapDisplay = mapDisplay;
    this.oi = oi;
    this.terminate = terminate;
    xVelocity = 0;
    yVelocity = 0;
    wVelocity = 0;
    this.slot = slot;

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

    if(DriverStation.getAlliance().isPresent())
    {
      DriverStation.Alliance alliance = DriverStation.getAlliance().get();
      if (alliance == Alliance.Red)
      {
        isRed = true;
      }
      else
      {
        isRed = false;
      }
    }


    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    xController.reset();
    yController.reset();
    thetaController.reset();
    aprilTagID = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Pose2d currentPose = localizer.getPose();

    if (oi.getDriverXButton())
    {
      slot = -1;
    }
    else if (oi.getDriverAButton())
    {
      slot = 0;
    }
    else if (oi.getDriverYButton())
    {
      slot = 1;
    }
    else if (oi.getDriverViewButton())
    {
      slot = 2;
    }

    if (aprilTagID == -1)
    {
      if (slot != 2)
      {
        aprilTagID = fieldMap.getBestReefTagID(currentPose);
        targetPose = fieldMap.getTagRelativePose(aprilTagID, slot, new Transform2d(0.65, 0, new Rotation2d(Math.PI)));
      }
      else
      {
        aprilTagID = fieldMap.getBestSourceTagID(currentPose, isRed);
        targetPose = fieldMap.getTagRelativePose(aprilTagID, 0, new Transform2d(0.75, 0, new Rotation2d(0)));
      }
    }
    SmartDashboard.putString("AlignTag", mapDisplay.aprilTagAssignments(aprilTagID));

    xError = Math.abs(targetPose.getX() - currentPose.getX());
    yError = Math.abs(targetPose.getY() - currentPose.getY());
    wError = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

    if (targetPose == null)
    {
      return;
    }

    xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    wVelocity = thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    xVelocity = MathUtil.clamp(xVelocity, -maximumLinearVelocity, maximumLinearVelocity);
    yVelocity = MathUtil.clamp(yVelocity, -maximumLinearVelocity, maximumLinearVelocity);
    wVelocity = MathUtil.clamp(wVelocity, -maximumRotationVelocity, maximumRotationVelocity);

    SmartDashboard.putNumber("AlignToTag/error", Math.sqrt(Math.pow(targetPose.minus(currentPose).getX(), 2) + Math.pow(targetPose.minus(currentPose).getY(), 2)));

    drivetrain.setTargetChassisSpeeds(
      ChassisSpeeds.fromFieldRelativeSpeeds (xVelocity, yVelocity, wVelocity, localizer.getPose().getRotation())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    aprilTagID = -1;
    targetPose = null;
    System.out.println("Terminated Global Align");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(terminate){
      return xError < 0.15 && yError < 0.13 && wError < 0.11;
    }
    return false;
  }
}
