// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AlgaeAutoGrab;
import frc.robot.commands.AlgaeEject;
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DetectElevatorHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.LoadAlgaeAuto;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreAlgaeAuto;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroAlgaePivot;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CommandStates;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeScore extends Command {
  /** Creates a new CenterCoralAndBarge. */
  public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, CommandStates state, AlgaePivot algaePivot, AprilTagFinder finder, Lidar lidar, int branchLevel) {
    int height;
    int slot;
    if (branchLevel == 1)
    {
      slot = 0;
    }
    else
    {
      slot = -1;
    }

    // tag 14 is the blue barge half from the blue side
    // tag 15 is the red barge half from the blue side, thus we do not use it
    // conversely for tag 4
    // and tag 5 is the red half from the red side

    Pose2d tag10Pose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag10ApproachPose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
    Pose2d tag10AlgaePose = map.getTagRelativePose(10, 0, new Transform2d(AutoConstants.algaeOffsetX, 0, new Rotation2d(Math.PI)));

    Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI))); 
    Pose2d tag21ApproachPose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
    Pose2d tag21AlgaePose = map.getTagRelativePose(21, 0, new Transform2d(AutoConstants.algaeOffsetX, 0, new Rotation2d(Math.PI)));

    Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

    Pose2d tag14Pose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.bargeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag5Pose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.bargeScoreOffsetX, 0, new Rotation2d(Math.PI)));

    Pose2d tag14BackPose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.algaeEndOffsetX - 0.5, 0, new Rotation2d()));
    Pose2d tag5BackPose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.algaeEndOffsetX - 0.5, 0, new Rotation2d()));

    Pose2d tag14EndPose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.algaeEndOffsetX, 0, new Rotation2d()));
    Pose2d tag5EndPose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.algaeEndOffsetX, 0, new Rotation2d()));

    Pose2d tag5LineUpPose = map.getTagRelativePose(5, 0, new Transform2d(0.5, 0, new Rotation2d()));
    Pose2d tag14LineUpPose = map.getTagRelativePose(14, 0, new Transform2d(0.5, 0, new Rotation2d()));

    Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
    tag10.blend_radius = AutoConstants.blendRadius;
    Point tag10Approach = new Point(tag10ApproachPose.getX(), tag10ApproachPose.getY());
    Point tag10Algae = new Point(tag10AlgaePose.getX(), tag10AlgaePose.getY());

    Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
    tag21.blend_radius = AutoConstants.blendRadius;
    Point tag21Approach = new Point(tag21ApproachPose.getX(), tag21ApproachPose.getY());
    Point tag21Algae = new Point(tag21AlgaePose.getX(), tag21AlgaePose.getY());

    Point tag14 = new Point(tag14Pose.getX(), tag14Pose.getY());
    tag14.blend_radius = AutoConstants.blendRadius;
    // Point tag15 = new Point(tag15Pose.getX(), tag15Pose.getY());
    // tag15.blend_radius = AutoConstants.blendRadius;
    // Point tag4 = new Point(tag4Pose.getX(), tag4Pose.getY());
    // tag4.blend_radius = AutoConstants.blendRadius;
    Point tag5 = new Point(tag5Pose.getX(), tag5Pose.getY());
    tag5.blend_radius = AutoConstants.blendRadius;

    Point tag5Back = new Point(tag5BackPose.getX(), tag5BackPose.getY());
    Point tag14Back = new Point(tag14BackPose.getX(), tag14BackPose.getY());

    Point tag5End = new Point(tag5EndPose.getX(), tag5EndPose.getY());
    Point tag14End = new Point(tag14EndPose.getX(), tag14EndPose.getY());

    Point tag5Offset = new Point(tag5LineUpPose.getX(), tag5LineUpPose.getY());
    Point tag14Offset = new Point(tag14LineUpPose.getX(), tag14LineUpPose.getY());

    ArrayList<Segment> segments = new ArrayList<Segment>();
    ArrayList<Segment> segments2 = new ArrayList<Segment>();
    ArrayList<Segment> segments3 = new ArrayList<Segment>();
    ArrayList<Segment> segments4 = new ArrayList<Segment>();

    Path path;
    Path path2;
    Path path3;
    Path path4;

    int tagID;

    if(isRed) 
    {
      segments.add(new Segment(start, tag10Approach, tag10ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      segments2.add(new Segment(tag10, tag10Algae, tag10AlgaePose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));

      segments3.add(new Segment(tag10, tag10Algae, tag5BackPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments3.add(new Segment(tag10Algae, tag5Offset, tag5BackPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));

      segments4.add(new Segment(tag5, tag5Back, tag5BackPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));


      path = new Path(segments, tag10ApproachPose.getRotation().getRadians());
      path2 = new Path(segments2, tag10AlgaePose.getRotation().getRadians());
      path3 = new Path(segments3, tag5BackPose.getRotation().getRadians());
      path4 = new Path(segments4, tag5EndPose.getRotation().getRadians());
      tagID = 10;
    }
    else 
    {
      segments.add(new Segment(start, tag21Approach, tag21ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      segments2.add(new Segment(tag21, tag21Algae, tag21AlgaePose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));

      segments3.add(new Segment(tag21, tag21Algae, tag14BackPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments3.add(new Segment(tag21Algae, tag14Offset, tag14BackPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));

      segments4.add(new Segment(tag14, tag14Back, tag14BackPose.getRotation().getRadians(), AutoConstants.reefApproachVelocity));

      path = new Path(segments, tag21ApproachPose.getRotation().getRadians());
      path2 = new Path(segments2, tag21AlgaePose.getRotation().getRadians());
      path3 = new Path(segments3, tag14BackPose.getRotation().getRadians());
      path4 = new Path(segments4, tag14EndPose.getRotation().getRadians());
      tagID = 21;
    }

    if(FieldMap.algaeHeight.get((isRed) ? 10 : 21) == 0) 
    {
      height = 5;
    }
    else 
    {
      height = 6;
    }

    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new WaitCommand(13),
          new SequentialCommandGroup(
          // load, drive to reef, and score
          new ParallelCommandGroup( 
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path, localizer),
            new CoralElevatorToHeight(elevator, 3, true) 
          ),
          new AlignToTagRelative(drivetrain, finder, state, tagID, slot), 
          new CoralElevatorToHeight(elevator, branchLevel, true),
          new ParallelRaceGroup( 
            new CoralElevatorToHeight(elevator, branchLevel, false),
            new SequentialCommandGroup(
              new ScoreCoral(endEffector),
              new WaitCommand(AutoConstants.elevatorDelay) 
            )
          ),

          //zero elevator and go back
          new ParallelRaceGroup(
            new ZeroElevator(elevator),
            new DrivePath(drivetrain, path2, localizer),
            AlgaeAutoGrab.create(algaePivot, endEffector),
            new WaitCommand(2.4)
          ),

          //run the algae grab, go to reef and align
          // new ParallelCommandGroup(
          //   AlgaeAutoGrab.create(algaePivot, endEffector),
          new SequentialCommandGroup(
            //new WaitCommand(0.8),
            new AlignToTagRelative(drivetrain, finder, state, tagID, 0),
            new ParallelRaceGroup(
              new WaitCommand(2),
              new LidarAlign(lidar, drivetrain, state)
            )
          ),
          // ),
          //drive to barge and score
          new ParallelCommandGroup(
            new DrivePath(drivetrain, path3, localizer),
            new CoralElevatorToHeight(elevator, 2, true)
          ),
          new CoralElevatorToHeight(elevator, 7, true),
          new ParallelRaceGroup( 
            new CoralElevatorToHeight(elevator, 7, false),
            new SequentialCommandGroup(
              new AlgaeEject(endEffector, algaePivot),
              new WaitCommand(AutoConstants.elevatorDelay),
              new ZeroAlgaePivot(algaePivot)
            )
          )
        )
      ),
      //no matter what zero and leave the climb area
      new ZeroElevator(elevator),
      new DrivePath(drivetrain, path4, localizer)
    );
  }
}
