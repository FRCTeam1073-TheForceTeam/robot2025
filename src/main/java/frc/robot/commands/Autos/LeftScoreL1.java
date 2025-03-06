package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.DrivePath;
import frc.robot.commands.LidarAlign;
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
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;

public class LeftScoreL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator, Lidar lidar)  
    {
        // TODO: Intermediate pose moves *away* from loading station? I would expect small X (back off) and larger Y (slide toward loading)
        Pose2d tag11Pose = map.getTagRelativePose(11, 0, new Transform2d(0.375, 0, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(11, 0, new Transform2d(2, 0.5, new Rotation2d(Math.PI)));
        Pose2d tag1Pose = map.getTagRelativePose(1, 0, new Transform2d(0.6, 0, new Rotation2d()));

        Pose2d tag20Pose = map.getTagRelativePose(20, 0, new Transform2d(0.375, 0, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(20, 0, new Transform2d(2, 0.5, new Rotation2d(Math.PI)));
        Pose2d tag13Pose = map.getTagRelativePose(13, 0, new Transform2d(0.6, 0, new Rotation2d()));

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag11 = new Point(tag11Pose.getX(), tag11Pose.getY());
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        Point tag1 = new Point(tag1Pose.getX(), tag1Pose.getY());

        Point tag20 = new Point(tag20Pose.getX(), tag20Pose.getY());
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        Point tag13 = new Point(tag13Pose.getX(), tag13Pose.getY());

        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();

        Path path1;
        Path path2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag11, tag11Pose.getRotation().getRadians(), 1.5));

            segments2.add(new Segment(tag11, redI1, redIntermediatePose.getRotation().getRadians(), 2));
            segments2.add(new Segment(redI1, tag1, tag1Pose.getRotation().getRadians(), 3));

            path1 = new Path(segments1, tag11Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag1Pose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag20, tag20Pose.getRotation().getRadians(), 1.5));

            segments2.add(new Segment(tag20, blueI1, blueIntermediatePose.getRotation().getRadians(), 2));
            segments2.add(new Segment(blueI1, tag13, tag13Pose.getRotation().getRadians(), 3));

            path1 = new Path(segments1, tag20Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag13Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            // TODO: Load and drive should be parallel. Every second counts.
            new LoadCoral(endEffector),
            new DrivePath(drivetrain, path1, localizer),
            // new LidarAlign(lidar, drivetrain),
            new CoralElevatorToHeight(elevator, 1, true),
            new ScoreCoral(endEffector),
            new ParallelCommandGroup(
                new ZeroElevator(elevator),
                new DrivePath(drivetrain, path2, localizer)
            )
        );
    }     
}
