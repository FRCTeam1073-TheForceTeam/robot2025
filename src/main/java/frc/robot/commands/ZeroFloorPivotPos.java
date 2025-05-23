// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickupPivot;
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroFloorPivotPos extends Command {

  FloorPickupPivot pivot;
  int count;

  /** Creates a new ZeroFloorPivotPos. */
  public ZeroFloorPivotPos(FloorPickupPivot pivot) {
    this.pivot = pivot;

    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivot.setRotatorVel(-5);
    // if(pivot.getRotatorLoad() > 9){
    //   count++;
    // }
    // else if(pivot.getRotatorLoad() <= 8 && count > 0){
    //   count--;
    // }
    pivot.setRotatorPos(0);
    SmartDashboard.putNumber("FloorPivot/Count", count);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count > 3;
  }
}
