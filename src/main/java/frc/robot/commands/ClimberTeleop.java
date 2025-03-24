package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OldOI;

public class ClimberTeleop extends Command
{
    Climber climber;
    OldOI oi;
    private double velocity;

  public ClimberTeleop(Climber climber, OldOI oi) 
  {
    this.climber = climber;
    this.oi = oi;
    addRequirements(climber);
  }

  @Override
  public void initialize() 
  {

  }

  @Override
  public void execute() 
  {
    velocity = 0;
    climber.setCommandedVelocity(velocity);
  }

  @Override
  public void end(boolean interrupted) 
  {

  }

  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
