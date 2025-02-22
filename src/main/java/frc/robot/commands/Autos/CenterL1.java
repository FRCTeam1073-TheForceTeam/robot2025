package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.TroughScoreCoral;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class CenterL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator)  
    {
        Pose2d tag10Pose = map.getTagRelativePose(10, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d tag21Pose = map.getTagRelativePose(21, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point tag10 = new Point(tag10Pose.getX(), tag10Pose.getY());
        Point tag21 = new Point(tag21Pose.getX(), tag21Pose.getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;
        if (isRed)
        {
            segments.add(new Segment(start, tag10, tag10Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag10Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag21, tag21Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag21Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            new DrivePath(drivetrain, path, localizer),
            new TroughScoreCoral(endEffector, elevator)
        );
    }   
}
