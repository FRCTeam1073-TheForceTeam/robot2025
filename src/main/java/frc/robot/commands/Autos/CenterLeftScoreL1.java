package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class CenterLeftScoreL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, double delay)  
    {
        Pose2d tag10Pose = map.getTagRelativePose(10, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d redEdgePose = new Pose2d(10, 1, new Rotation2d());
        Pose2d tag1Pose = map.getTagRelativePose(1, 0, new Transform2d(0.75, 0, new Rotation2d()));

        Pose2d tag21Pose = map.getTagRelativePose(21, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d blueEdgePose = new Pose2d(8, 7, new Rotation2d(Math.PI)); 
        Pose2d tag13Pose = map.getTagRelativePose(13, 0, new Transform2d(0.75, 0, new Rotation2d()));
        
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        Point redEdge = new Point(redEdgePose.getX(), redEdgePose.getY());
        Point tag1 = new Point(tag1Pose.getX(), tag1Pose.getY());

        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        Point blueEdge = new Point(blueEdgePose.getX(), blueEdgePose.getY());
        Point tag13 = new Point(tag13Pose.getX(), tag13Pose.getY());



        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        Path path1;
        Path path2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), 1));

            segments2.add(new Segment(tag10, redEdge, redEdgePose.getRotation().getRadians(), 1));
            segments2.add(new Segment(redEdge, tag1, tag1Pose.getRotation().getRadians(), 1));

            path1 = new Path(segments1, tag10Pose.getRotation().getRadians());
            path2 = new Path(segments2, redEdgePose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), 1));

            segments2.add(new Segment(tag21, blueEdge, blueEdgePose.getRotation().getRadians(), 1));
            segments2.add(new Segment(blueEdge, tag13, tag13Pose.getRotation().getRadians(), 1));

            path1 = new Path(segments1, tag21Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag13Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            new DrivePath(drivetrain, path1, localizer),
            new WaitCommand(delay),
            new DrivePath(drivetrain, path2, localizer)
        );
    }   
}
