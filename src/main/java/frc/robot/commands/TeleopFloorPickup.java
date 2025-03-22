// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FloorPickup;
import frc.robot.subsystems.OI;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopFloorPickup extends Command {
  /** Creates a new TeleopFloorPickup. */
  FloorPickup floorPickup;
  OI oi;
  private int togglePositions = -1;
  private double positionZero = 0; //Initialize Position
  private double positionOne = 0.2; //Floor Position
  private double positionTwo = 0.1; //Stow Position / Score Position

  public TeleopFloorPickup(FloorPickup floorPickup, OI oi) {
    this.floorPickup = floorPickup;
    this.oi = oi;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Zero Position
    togglePositions = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO ADD BUTTON
    if(buttonPush){
      togglePositions++;
      if(togglePositions > 2){
        togglePositions = 0;
      }
    }
    if(togglePositions == 0){
      floorPickup.setPivotPosition(positionZero);
    }
    else if(togglePositions == 1){
      floorPickup.setPivotPosition(positionOne);
    }
    else if(togglePositions == 2){
      floorPickup.setPivotPosition(positionTwo);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
