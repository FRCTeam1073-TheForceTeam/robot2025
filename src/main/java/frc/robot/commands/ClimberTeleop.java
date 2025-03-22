package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.OI;

public class ClimberTeleop extends Command
{
    Climber climber;
    OI oi;
    private double velocity;

  public ClimberTeleop(Climber climber, OI oi) 
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
    if (oi.getOperatorLeftBumper()){
      velocity = 10;
    }
    else if (oi.getOperatorRightBumper()){
      velocity = -10;
    }
    else{
      if(!oi.getOperatorAButton() && !oi.getOperatorBButton() && !oi.getOperatorMenuButton()){
        if(!climber.getIsAtZero()){
          velocity = -climber.getMotorPosition();
          if(velocity < 0){
            MathUtil.clamp(velocity, -10, -5);
          } else if(velocity > 0){
            MathUtil.clamp(velocity, 5, 10);
          }
        }
      }
      velocity = 0;
    }
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
