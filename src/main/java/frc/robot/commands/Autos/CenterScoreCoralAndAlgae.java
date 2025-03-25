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
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class CenterScoreCoralAndAlgae 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar, AprilTagFinder finder, int branchLevel)  
    {
        int slot;
        if (branchLevel == 1)
        {
            slot = 0;
        }
        else
        {
            slot = -1;
        }
        Pose2d tag10Pose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag10ApproachPose = map.getTagRelativePose(10, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag10IntermediatePose = map.getTagRelativePose(10, 0, new Transform2d(AutoConstants.intermediateOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag10AlgaePose = map.getTagRelativePose(10, 0, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag5Pose = map.getTagRelativePose(5, 0, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));


        Pose2d tag21Pose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI))); 
        Pose2d tag21ApproachPose = map.getTagRelativePose(21, slot, new Transform2d(AutoConstants.scoreApproachOffsetX, -AutoConstants.scoreApproachOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag21IntermediatePose = map.getTagRelativePose(21, 0, new Transform2d(AutoConstants.intermediateOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag21AlgaePose = map.getTagRelativePose(21, 0, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag14Pose = map.getTagRelativePose(14, 0, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        tag10.blend_radius = AutoConstants.blendRadius;
        Point tag10Approach = new Point(tag10ApproachPose.getX(), tag10ApproachPose.getY());
        Point tag10I = new Point(tag10IntermediatePose.getX(), tag10IntermediatePose.getY());
        Point tag10A = new Point(tag10AlgaePose.getX(), tag10AlgaePose.getY());
        Point tag5 = new Point(tag5Pose.getX(), tag5Pose.getY());

        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        tag21.blend_radius = AutoConstants.blendRadius;
        Point tag21Approach = new Point(tag21ApproachPose.getX(), tag21ApproachPose.getY());
        Point tag21I = new Point(tag21IntermediatePose.getX(), tag21IntermediatePose.getY());
        Point tag21A = new Point(tag21AlgaePose.getX(), tag21AlgaePose.getY());
        Point tag14 = new Point(tag14Pose.getX(), tag14Pose.getY());


        ArrayList<Segment> segments0 = new ArrayList<Segment>();
        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();

        Path path0;
        Path path1;
        Path path2;

        int tagID;

        if (isRed)
        {
            segments0.add(new Segment(start, tag10Approach, tag10ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments1.add(new Segment(tag10, tag10I, tag10IntermediatePose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag10A, tag5, tag5Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path0 = new Path(segments0, tag10ApproachPose.getRotation().getRadians());
            path1 = new Path(segments1, tag10IntermediatePose.getRotation().getRadians());
            path2 = new Path(segments2, tag5Pose.getRotation().getRadians());

            tagID = 10;
        }
        else
        {
            segments0.add(new Segment(start, tag21Approach, tag21ApproachPose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments1.add(new Segment(tag21, tag21I, tag21IntermediatePose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            segments2.add(new Segment(tag21A, tag14, tag14Pose.getRotation().getRadians(), AutoConstants.scoringAlignmentVelocity));

            path0 = new Path(segments0, tag21ApproachPose.getRotation().getRadians());
            path1 = new Path(segments1, tag21IntermediatePose.getRotation().getRadians());
            path2 = new Path(segments2, tag14Pose.getRotation().getRadians());

            tagID = 21;
        }
        

        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new LoadCoral(endEffector),
                new DrivePath(drivetrain, path0, localizer)
            ),
            new AlignToTagRelative(drivetrain, finder, tagID, slot),
            new CoralElevatorToHeight(elevator, branchLevel, true),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                                              new WaitCommand(0.5))),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path1, localizer)
            ),
            new AlignToTagRelative(drivetrain, finder, tagID, 0),
            // algae stuff
            new DrivePath(drivetrain, path2, localizer)
        );
    }
}
