// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickupCollect;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorPickupCollectTeleop extends Command {
  FloorPickupCollect collect;
  double velocity;

  /** Creates a new FloorPickupCollectTeleop. */
  public FloorPickupCollectTeleop(FloorPickupCollect floorPickupCollect) {
    collect = floorPickupCollect;
    velocity = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(collect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(collect.getLoad() <= 20) {
      velocity = 25;
    }
    else {
      velocity = 0;
    }
    collect.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
