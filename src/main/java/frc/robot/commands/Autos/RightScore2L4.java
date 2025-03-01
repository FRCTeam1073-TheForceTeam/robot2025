package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class RightScore2L4
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer)  
    {
        Pose2d tag9Pose = map.getTagRelativePose(9, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d redIntermediatePose = map.getTagRelativePose(9, 0, new Transform2d(2, -0.5, new Rotation2d(Math.PI)));
        Pose2d tag2Pose = map.getTagRelativePose(2, 0, new Transform2d(0.75, 0, new Rotation2d()));

        Pose2d tag22Pose = map.getTagRelativePose(22, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d blueIntermediatePose = map.getTagRelativePose(22, 0, new Transform2d(2, -0.5, new Rotation2d(Math.PI)));
        Pose2d tag12Pose = map.getTagRelativePose(12, 0, new Transform2d(0.75, 0, new Rotation2d()));


        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());

        Point tag9 = new Point(tag9Pose.getX(), tag9Pose.getY());
        Point redI1 = new Point(redIntermediatePose.getX(), redIntermediatePose.getY());
        Point tag2 = new Point(tag2Pose.getX(), tag2Pose.getY());

        Point tag22 = new Point(tag22Pose.getX(), tag22Pose.getY());
        Point blueI1 = new Point(blueIntermediatePose.getX(), blueIntermediatePose.getY());
        Point tag12 = new Point(tag12Pose.getX(), tag12Pose.getY());


        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;

        
        if (isRed)
        {
            segments.add(new Segment(start, tag9, tag9Pose.getRotation().getRadians(), 1));
            segments.add(new Segment(tag9, redI1, redIntermediatePose.getRotation().getRadians(), 1));
            segments.add(new Segment(redI1, tag2, tag2Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag2Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag22, tag22Pose.getRotation().getRadians(), 1));
            segments.add(new Segment(tag22, blueI1, blueIntermediatePose.getRotation().getRadians(), 1));
            segments.add(new Segment(blueI1, tag12, tag12Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag12Pose.getRotation().getRadians());
        }
        

        return new DrivePath(drivetrain, path, localizer);
    } 
}
