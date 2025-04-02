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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CreepToReef;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadAlgaeAuto;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreAlgaeAuto;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CenterCoralAndBarge extends Command {
  /** Creates a new CenterCoralAndBarge. */
  public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, AlgaePivot algaePivot, Lidar lidar, int branchLevel) {
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

    Pose2d tag10Pose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));

    Pose2d tag14Pose = map.getTagRelativePose(14, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag15Pose = map.getTagRelativePose(15, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI))); //TODO: implement a way to pick which side go to
    Pose2d tag4Pose = map.getTagRelativePose(4, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));
    Pose2d tag5Pose = map.getTagRelativePose(5, slot, new Transform2d(AutoConstants.algaeScoreOffsetX, 0, new Rotation2d(Math.PI)));

    
    Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
    Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
    tag10.blend_radius = AutoConstants.blendRadius;
    Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
    tag21.blend_radius = AutoConstants.blendRadius;
    Point tag14 = new Point(tag14Pose.getX(), tag14Pose.getY());
    tag14.blend_radius = AutoConstants.blendRadius;
    Point tag15 = new Point(tag15Pose.getX(), tag15Pose.getY());
    tag15.blend_radius = AutoConstants.blendRadius;
    Point tag4 = new Point(tag4Pose.getX(), tag4Pose.getY());
    tag4.blend_radius = AutoConstants.blendRadius;
    Point tag5 = new Point(tag5Pose.getX(), tag5Pose.getY());
    tag5.blend_radius = AutoConstants.blendRadius;

    ArrayList<Segment> segments = new ArrayList<Segment>();
    ArrayList<Segment> segments2 = new ArrayList<Segment>();
    Path path;
    Path path2;

    if(isRed) {
      segments.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments2.add(new Segment(tag10, tag5, tag5Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      path = new Path(segments, tag10Pose.getRotation().getRadians());
      path2 = new Path(segments2, tag4Pose.getRotation().getRadians());

    }
    else {
      segments.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));
      segments2.add(new Segment(tag21, tag14, tag14Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

      path = new Path(segments, tag21Pose.getRotation().getRadians());
      path2 = new Path(segments2, tag14Pose.getRotation().getRadians());
    }

    if(FieldMap.algaeHeight.get((isRed) ? 10 : 21) == 0) {
      height = 5;
    }
    else {
      height = 6;
    }


    return new SequentialCommandGroup(
      new ParallelCommandGroup(
          new LoadCoral(endEffector),
          new DrivePath(drivetrain, path, localizer)
      ),
      // new LidarAlign(lidar, drivetrain),
      new CoralElevatorToHeight(elevator, branchLevel, true),
      new ParallelRaceGroup(
        new CoralElevatorToHeight(elevator, branchLevel, false),
        new SequentialCommandGroup(
        new CreepToReef(drivetrain, endEffector, -1).withTimeout(3.0),
        new ScoreCoral(endEffector)),
      new WaitCommand(0.5),
      new ZeroElevator(elevator)),
      new ParallelCommandGroup(
        new CoralElevatorToHeight(elevator, height, true),
        new LoadAlgaeAuto(algaePivot)
      ),
      new ZeroElevator(elevator),
      new DrivePath(drivetrain, path2, localizer),
      new CoralElevatorToHeight(elevator, 7, true),
      new ScoreAlgaeAuto(algaePivot),
      new ZeroElevator(elevator));
  }
}
