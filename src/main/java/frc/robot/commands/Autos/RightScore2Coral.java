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
import frc.robot.commands.DrivePath;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.Path;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ZeroElevator;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class RightScore2Coral 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, int branchLevel)  
    {
        Pose2d tag9RightPose = map.getTagRelativePose(9, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag9LeftPose = map.getTagRelativePose(9, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(9, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 0, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Pose2d tag22RightPose = map.getTagRelativePose(22, 1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d tag22LeftPose = map.getTagRelativePose(22, -1, new Transform2d(AutoConstants.scoreOffsetX, 0, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(22, 0, new Transform2d(AutoConstants.intermediateOffsetX, -AutoConstants.intermediateOffsetY, new Rotation2d(Math.PI)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 0, new Transform2d(AutoConstants.loadOffsetX, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag9R = new Point(tag9RightPose.getX(), tag9RightPose.getY());
        tag9R.blend_radius = AutoConstants.blendRaidus;
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        redI1.blend_radius = AutoConstants.blendRaidus;
        Point tag1 = new Point(tag2Pose.getX(), tag2Pose.getY());
        tag1.blend_radius = AutoConstants.blendRaidus;
        Point tag9L = new Point(tag9LeftPose.getX(), tag9LeftPose.getY());
        tag9L.blend_radius = AutoConstants.blendRaidus;

        Point tag22R = new Point(tag22RightPose.getX(), tag22RightPose.getY());
        tag22R.blend_radius = AutoConstants.blendRaidus;
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        blueI1.blend_radius = AutoConstants.blendRaidus;
        Point tag13 = new Point(tag12Pose.getX(), tag12Pose.getY());
        tag13.blend_radius = AutoConstants.blendRaidus;
        Point tag22L = new Point(tag22LeftPose.getX(), tag22LeftPose.getY());
        tag22L.blend_radius = AutoConstants.blendRaidus;

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        ArrayList<Segment> segments3 = new ArrayList<Segment>();

        Path path1;
        Path path2;
        Path path3;

        if (isRed)
        {
            segments1.add(new Segment(start, tag9R, tag9RightPose.getRotation().getRadians(), 1.5));

            segments2.add(new Segment(tag9R, redI1, redIntermediatePose.getRotation().getRadians(), 2));
            segments2.add(new Segment(redI1, tag1, tag2Pose.getRotation().getRadians(), 2));

            segments3.add(new Segment(tag1, redI1, redIntermediatePose.getRotation().getRadians(), 2));
            segments3.add(new Segment(redI1, tag9L, tag9LeftPose.getRotation().getRadians(), 1.5));

            path1 = new Path(segments1, tag9RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag2Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag9LeftPose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag22R, tag22RightPose.getRotation().getRadians(), 1.5));

            segments2.add(new Segment(tag22R, blueI1, blueIntermediatePose.getRotation().getRadians(), 2));
            segments2.add(new Segment(blueI1, tag13, tag12Pose.getRotation().getRadians(), 2));

            segments3.add(new Segment(tag13, blueI1, blueIntermediatePose.getRotation().getRadians(), 2));
            segments3.add(new Segment(blueI1, tag22L, tag22LeftPose.getRotation().getRadians(), 1.5));

            path1 = new Path(segments1, tag22RightPose.getRotation().getRadians());
            path2 = new Path(segments2, tag12Pose.getRotation().getRadians());
            path3 = new Path(segments3, tag22LeftPose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            // TODO: Load and drive should be parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path1, localizer),
                        new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
                                   new SequentialCommandGroup(new ScoreCoral(endEffector),
                                       new WaitCommand(0.5))),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path2, localizer)
            ),
            // TODO: Consider using wait in stead of using load as wait.
            // TODO: Load and drive should be parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path3, localizer),
            new ParallelRaceGroup( new CoralElevatorToHeight(elevator, branchLevel, false),
            new SequentialCommandGroup(new ScoreCoral(endEffector),
                new WaitCommand(0.5))),
            new ZeroElevator(elevator)
        );
    }     
}
