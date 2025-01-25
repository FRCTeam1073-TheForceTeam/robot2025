package frc.robot.commands;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.OI; -- oi doesnt exist yet in robot2025 :sad-face:


public class ClimberClawTeleop extends Command {

Climber m_climber;
// OI m_oi;

public ClimberClawTeleop(Climber climber) {
    m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}

