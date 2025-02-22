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

public class LeftL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer)  
    {
        Pose2d tag11Pose = map.getTagRelativePose(11, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d tag20Pose = map.getTagRelativePose(20, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point tag11 = new Point(tag11Pose.getX(), tag11Pose.getY());
        Point tag20 = new Point(tag20Pose.getX(), tag20Pose.getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;
        if (isRed)
        {
            segments.add(new Segment(start, tag11, tag11Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag11Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag20, tag20Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag20Pose.getRotation().getRadians());
        }
        

        return new DrivePath(drivetrain, path, localizer);
    }     
}
