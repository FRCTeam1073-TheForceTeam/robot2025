// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickupCollect;
import frc.robot.subsystems.FloorPickupPivot;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ProcessorScore extends Command {

  private final double pos = 9.03;

  FloorPickupCollect floorPickupCollect;
  FloorPickupPivot floorPickupPivot;

  public ProcessorScore(FloorPickupCollect qfloorPickupCollect, FloorPickupPivot qfloorPickupPivot) {
    floorPickupCollect = qfloorPickupCollect;
    floorPickupPivot = qfloorPickupPivot;

    addRequirements(floorPickupCollect , floorPickupPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    floorPickupPivot.setRotatorPos(pos);
    if (floorPickupCollect.getLoad() <= 20){
      floorPickupCollect.setVelocity(15);
    }
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
