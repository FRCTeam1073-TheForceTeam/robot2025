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

public class CenterRightScoreL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, double delay)  
    {
        Pose2d tag10Pose = map.getTagRelativePose(10, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d tag3Pose = map.getTagRelativePose(3, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI / 2)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 0, new Transform2d(0.75, 0, new Rotation2d()));

        Pose2d tag21Pose = map.getTagRelativePose(21, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d tag16Pose = map.getTagRelativePose(16, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI / 2)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 0, new Transform2d(0.75, 0, new Rotation2d()));
        
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        Point tag3 = new Point(tag3Pose.getX(), tag3Pose.getY());
        Point tag2 = new Point(tag2Pose.getX(), tag2Pose.getY());

        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());
        Point tag16 = new Point(tag16Pose.getX(), tag16Pose.getY());
        Point tag12 = new Point(tag12Pose.getX(), tag12Pose.getY());



        ArrayList<Segment> segments1 = new ArrayList<Segment>();
        ArrayList<Segment> segments2 = new ArrayList<Segment>();
        Path path1;
        Path path2;

        if (isRed)
        {
            segments1.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), 1));

            segments2.add(new Segment(tag10, tag3, tag3Pose.getRotation().getRadians(), 1));
            segments2.add(new Segment(tag3, tag2, tag2Pose.getRotation().getRadians(), 1));

            path1 = new Path(segments1, tag10Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag3Pose.getRotation().getRadians());
        }
        else
        {
            segments1.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), 1));

            segments2.add(new Segment(tag21, tag16, tag16Pose.getRotation().getRadians(), 1));
            segments2.add(new Segment(tag16, tag12, tag12Pose.getRotation().getRadians(), 1));

            path1 = new Path(segments1, tag21Pose.getRotation().getRadians());
            path2 = new Path(segments2, tag12Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            new DrivePath(drivetrain, path1, localizer),
            new WaitCommand(delay),
            new DrivePath(drivetrain, path2, localizer)
        );
    }   
}
