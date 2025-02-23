package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.TroughScoreCoral;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class RightL1 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, FieldMap map, Localizer localizer, CoralEndeffector endEffector, CoralElevator elevator) 
    {
        Pose2d tag9Pose = map.getTagRelativePose(9, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Pose2d tag22Pose = map.getTagRelativePose(22, 0, new Transform2d(0.75, 0, new Rotation2d(Math.PI)));
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point tag9 = new Point(tag9Pose.getX(), tag9Pose.getY());
        Point tag22 = new Point(tag22Pose.getX(), tag22Pose.getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();
        Path path;
        if (isRed)
        {
            segments.add(new Segment(start, tag9, tag9Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag9Pose.getRotation().getRadians());
        }
        else
        {
            segments.add(new Segment(start, tag22, tag22Pose.getRotation().getRadians(), 1));

            path = new Path(segments, tag22Pose.getRotation().getRadians());
        }
        

        return new SequentialCommandGroup(
            new DrivePath(drivetrain, path, localizer),
            new TroughScoreCoral(endEffector, elevator)
        );
    }  
}
