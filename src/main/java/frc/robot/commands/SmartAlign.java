// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartAlign extends SequentialCommandGroup {
  public static Command create(Drivetrain drivetrain, Localizer localizer, int slot, Lidar lidar, AprilTagFinder aprilTagFinder, FieldMap fieldMap, MapDisplay mapDisplay, OI oi){
    return new SequentialCommandGroup(
      new AlignToTagRelative(drivetrain, aprilTagFinder, slot, localizer, fieldMap, mapDisplay, oi),
      new LidarAlign(lidar, drivetrain)
    );
  }
}
