package frc.robot.commands.Autos;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.DrivePath;
import frc.robot.commands.Path;
import frc.robot.commands.Path.Point;
import frc.robot.commands.Path.Segment;
import frc.robot.commands.ZeroClaw;
import frc.robot.commands.ZeroLift;
import frc.robot.subsystems.ClimberClaw;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Localizer;

public class GenericL0 
{
    public static Command create(boolean isRed, Drivetrain drivetrain, Localizer localizer, ClimberClaw claw, ClimberLift lift)
    {
        int allianceSign;

        if (isRed)
        {
            allianceSign = 1;
        }
        else 
        {
            allianceSign = -1;
        }

        Point start = new Point(localizer.getPose().getX(), localizer.getPose().getY());
        Point end = new Point(localizer.getPose().getX() + 1.0 * allianceSign, localizer.getPose().getY());

        ArrayList<Segment> segments = new ArrayList<Segment>();

        segments.add(new Segment(start, end, 0, 1));

        Path path = new Path(segments, 0);

        return new ParallelCommandGroup(
            new DrivePath(drivetrain, path, localizer),
            new ZeroClaw(claw),
            new ZeroLift(lift)
        );
    }    
}
