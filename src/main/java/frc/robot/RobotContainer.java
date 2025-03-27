// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.AlignToTagRelative;
import frc.robot.commands.CancelLoadCoral;
import frc.robot.commands.CANdleObserver;
import frc.robot.commands.ClimberTeleop;
import frc.robot.commands.CoralElevatorTeleop;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CoralEndeffectorTeleop;
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.LidarAlign;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.RemoveAlgae;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.Autos.AutoCenterStart;
import frc.robot.commands.Autos.AutoLeftStart;
import frc.robot.commands.Autos.AutoRightStart;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Lidar;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.OpenMVCAN;
import frc.robot.subsystems.CANdleControl;

public class RobotContainer implements Consumer<String> // need the interface for onChange
{
  //private final Drivetrain m_drivetrain = new Drivetrain();
  private final OpenMVCAN m_mvCan = new OpenMVCAN(1);

  private boolean isRed;
  private int level;
  private boolean isRainbow = false;

  public boolean haveInitStartPos = false;

  public RobotContainer() 
  {
    configureBindings();
  }

  private void configureBindings() {
  } 

  public void autonomousInit() {
  }

  public Command getAutonomousCommand() 
  {
    return new WaitCommand(100);
  }

  public void printAllFalseDiagnostics()
  {
    boolean isDisabled = DriverStation.isDisabled();
    boolean allOK = true;
    // Set allOK to the results of the printDiagnostics method for each subsystem, separated by &&
    allOK = true;
    //TODO: Add each subsystem
    SmartDashboard.putBoolean("Engine light", allOK);
  }

  public Command getTeleopCommand()
  {
    return null;
  }

  public Command getDisabledCommand() 
  {
    return null;
  }

  public void disabledInit() 
  {
    
    haveInitStartPos = false;
  }

  public boolean disabledPeriodic() 
  {
    return false;
  }

  @Override
  public void accept(String t) // gets called every time the selected position changes so the start position is reinitialized
  {
    haveInitStartPos = false;  
  }
}
