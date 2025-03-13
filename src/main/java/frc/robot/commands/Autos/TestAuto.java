package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;

public class TestAuto 
{
    public static Command create(Drivetrain drivetrain, Localizer localizer, FieldMap map)
    {
        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point point1 = new Point(map.getTagRelativePose(20, 0, new Transform2d(2, 0, new Rotation2d())).getX(), map.getTagRelativePose(20, 0, new Transform2d(2, 0, new Rotation2d())).getY()); 
        Point point2 = new Point(map.getTagRelativePose(18, 0, new Transform2d(2, 0, new Rotation2d())).getX(), map.getTagRelativePose(18, 0, new Transform2d(2, 0, new Rotation2d())).getY());
        Point point3 = new Point(map.getTagRelativePose(22, 0, new Transform2d(2, 0, new Rotation2d())).getX(), map.getTagRelativePose(22, 0, new Transform2d(2, 0, new Rotation2d())).getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();
        segments.add(new Segment(start, point1, 0, 2));
        segments.add(new Segment(point1, point2, 0, 2));
        segments.add(new Segment(point2, point3, 0, 2));
        segments.add(new Segment(point3, start, 0, 2));
        // segments.add(new Segment(start, point1, 0, 2));

        point2.blend_radius = 1;

        Path path = new Path(segments, 0);
        return new SequentialCommandGroup(new DrivePath(drivetrain, path, localizer));
    }
}
