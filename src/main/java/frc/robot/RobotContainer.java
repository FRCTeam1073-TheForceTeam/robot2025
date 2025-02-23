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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToTag;
import frc.robot.commands.CancelLoadCoral;
import frc.robot.commands.ClimberTeleop;
import frc.robot.commands.CoralElevatorTeleop;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CoralEndeffectorTeleop;
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TroughScoreCoral;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.Autos.AutoCenterLeftStart;
import frc.robot.commands.Autos.AutoCenterRightStart;
import frc.robot.commands.Autos.AutoLeftStart;
import frc.robot.commands.Autos.AutoRightStart;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;

public class RobotContainer implements Consumer<String> // need the interface for onChange
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Field2d m_field = new Field2d();
  private final FieldMap m_fieldMap = new FieldMap();
  private final Localizer m_localizer = new Localizer(m_drivetrain, m_fieldMap, m_aprilTagFinder);
  private final MapDisplay m_MapDisplay = new MapDisplay(m_drivetrain, m_localizer, m_fieldMap);
  private final CoralElevator m_coralElevator = new CoralElevator();
  private final CoralEndeffector m_coralEndeffector = new CoralEndeffector();
  private final Climber m_climber = new Climber();

  private final ZeroElevator cmd_zeroElevator = new ZeroElevator(m_coralElevator);
  private final CoralElevatorTeleop cmd_coralElevatorTeleop = new CoralElevatorTeleop(m_coralElevator, m_OI);
  private final CoralEndeffectorTeleop cmd_coralEndeffectorTeleop = new CoralEndeffectorTeleop(m_coralEndeffector, m_OI);
  private final LoadCoral cmd_loadCoral = new LoadCoral(m_coralEndeffector);
  private final ScoreCoral cmd_scoreCoral = new ScoreCoral(m_coralEndeffector);
  private final CoralElevatorToHeight cmd_coralElevatorToL2 = new CoralElevatorToHeight(m_coralElevator, m_OI, 2);
  private final CoralElevatorToHeight cmd_coralElevatorToL3 = new CoralElevatorToHeight(m_coralElevator, m_OI, 3);
  private final CoralElevatorToHeight cmd_coralElevatorToL4 = new CoralElevatorToHeight(m_coralElevator, m_OI, 4);
  private final TroughScoreCoral cmd_troughScoreCoral = new TroughScoreCoral(m_coralEndeffector, m_coralElevator);
  private final CancelLoadCoral cmd_cancelLoadCoral = new CancelLoadCoral(m_coralEndeffector);
  private final AlignToTag cmd_alignToTag = new AlignToTag(m_drivetrain, m_localizer, m_fieldMap, m_OI);
  private final ClimberTeleop cmd_climberTeleop = new ClimberTeleop(m_climber, m_OI);
  private final ZeroClimber cmd_zeroClimber = new ZeroClimber(m_climber);
  private final EngageClimber cmd_engageClimber = new EngageClimber(m_climber);
  private final DisengageClimber cmd_disengageClimber = new DisengageClimber(m_climber);

  private final TeleopDrive cmd_teleopDrive = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder, m_localizer);

  private boolean isRed;
  private int level;

  public boolean haveInitStartPos = false;

  private final SendableChooser<String> m_positionChooser = new SendableChooser<>();
  private static final String noPosition = "No Position";
  private static final String rightPosition = "Right Auto";
  private static final String leftPosition = "Left Auto";
  private static final String centerLeftPosition = "Center Left Auto";
  private static final String centerRightPosition = "Center Right Auto";
  
  private final SendableChooser<String> m_levelChooser = new SendableChooser<>();
  private static final String testLevel = "Test Level";
  private static final String noLevelAuto = "No Level";
  private static final String level0 = "Level 0";
  private static final String level1 = "Level 1";
  private static final String level2 = "Level 2";
  private static final String level3 = "Level 3";
  private static final String level4 = "Level 4";
  private static final String zeroClawAndLift = "Zero Claw And Lift";

  private double autoDelay = 0;

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, cmd_teleopDrive);
    CommandScheduler.getInstance().setDefaultCommand(m_coralElevator, cmd_coralElevatorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_coralEndeffector, cmd_coralEndeffectorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_climber, cmd_climberTeleop);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_localizer);

    m_positionChooser.setDefaultOption("No Position", noPosition);
    m_positionChooser.addOption("Right Position", rightPosition);
    m_positionChooser.addOption("Left Position", leftPosition);
    m_positionChooser.addOption("Center Left Position", centerLeftPosition);
    m_positionChooser.addOption("Center Right Position", centerRightPosition);
    m_positionChooser.addOption("Zero Claw and Lift", zeroClawAndLift);

    m_levelChooser.setDefaultOption("No Level", noLevelAuto);
    m_levelChooser.addOption("Test Auto", testLevel);
    m_levelChooser.addOption("Level 0", level0);
    m_levelChooser.addOption("Level 1", level1);
    m_levelChooser.addOption("Level 2", level2);
    m_levelChooser.addOption("Level 3", level3);
    m_levelChooser.addOption("Level 4", level4);

    SmartDashboard.putData("Position Chooser", m_positionChooser);
    SmartDashboard.putData("Level Chooser", m_levelChooser);

    SmartDashboard.getNumber("Auto Delay", autoDelay);

    m_positionChooser.onChange(this::accept); // this is so we can reset the start position


    configureBindings();
  }

  private void configureBindings() {
    Trigger disengageClimber = new Trigger(m_OI::getOperatorAButton);
      disengageClimber.onTrue(cmd_disengageClimber);
    Trigger engageClimber = new Trigger(m_OI::getOperatorBButton);
      engageClimber.onTrue(cmd_engageClimber);
    Trigger zeroClimber = new Trigger(m_OI::getOperatorMenuButton);
      zeroClimber.onTrue(cmd_zeroClimber);
    Trigger zeroElevator = new Trigger(m_OI::getOperatorLeftJoystickPress);
      zeroElevator.onTrue(cmd_zeroElevator);

    Trigger loadCoral = new Trigger(m_OI::getOperatorXButton);
      loadCoral.onTrue(cmd_loadCoral);

    Trigger scoreCoral = new Trigger(m_OI::getOperatorYButton);
      scoreCoral.onTrue(cmd_scoreCoral);

    // Trigger zeroClawAndLift = new Trigger(m_OI::getOperatorRightJoystickPress);
    //   zeroClawAndLift.onTrue(ZeroClawAndLift.create(m_climberClaw, m_climberLift));
    Trigger elevatorL2 = new Trigger(m_OI :: getOperatorDPadRight);
      elevatorL2.whileTrue(cmd_coralElevatorToL2);

    Trigger elevatorL3 = new Trigger(m_OI::getOperatorDPadDown);
      elevatorL3.whileTrue(cmd_coralElevatorToL3);
    
    Trigger elevatorL4 = new Trigger(m_OI::getOperatorDPadLeft);
      elevatorL4.whileTrue(cmd_coralElevatorToL4);

    Trigger troughScore = new Trigger(m_OI::getOperatorDPadUp);
      troughScore.onTrue(cmd_troughScoreCoral);

    Trigger cancelLoadCoral = new Trigger(m_OI::getOperatorRightTrigger);
      cancelLoadCoral.onTrue(cmd_cancelLoadCoral);

    Trigger alignToTag = new Trigger(m_OI::getDriverAlignToTag);
      alignToTag.whileTrue(cmd_alignToTag);
  }

  public void autonomousInit()
  {

  }

  public Command getAutonomousCommand() 
  {
   // return Commands.print("No autonomous command configured");
   // -1 to indicate no auto select
    switch(m_levelChooser.getSelected())
    {
      case noLevelAuto:
        level = -1;
        break;
      case level0:
        level = 0;
        break;
      case level1:
        level = 1;
        break;
      case level2:
        level = 2;
        break;
      case level3:
        level = 3;
        break;
      case level4:
        level = 4;                  
        break;
      case testLevel:
        level = 99;
        break;
      default:
        level = -1;
        break;
    }


    switch(m_positionChooser.getSelected())
    {
      // case zeroClawAndLift:
      //   return ZeroClawAndLift.create(m_climberClaw, m_climberLift);
      case noPosition:
        return null;
      case leftPosition:
        return AutoLeftStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator);
      case rightPosition:
        return AutoRightStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator);
      case centerLeftPosition:
        return AutoCenterLeftStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, autoDelay);
      case centerRightPosition:
        return AutoCenterRightStart.create(level, isRed, m_drivetrain, m_localizer, m_fieldMap, m_climber, m_coralEndeffector, m_coralElevator, autoDelay);
      default:
        return null;
    }
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

  public boolean findStartPos() 
  {
      if(!DriverStation.getAlliance().isPresent() || m_positionChooser.getSelected().equals("No Position")) {
        return false;
      }

      //create a bool for pose is set
      double centerY = 4.026;
      int allianceSign = 1;
      String initPosition = m_positionChooser.getSelected();
      
      double centerX = 8.774; // this is the default 
      double startLineOffset = 12.227 -8.774 - 2.24; //id 10 x value - center x value - offset from reef to startline
      Pose2d startPos = new Pose2d();
      SmartDashboard.putString("Alliance", "None");
  
      if(DriverStation.getAlliance().isPresent())
      {
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        if(alliance == Alliance.Blue) {
          allianceSign = -1;
        }
        if (initPosition.equals(leftPosition)) {
          centerY -= allianceSign * 2.013;
        }
        else if(initPosition.equals(rightPosition)) {
          centerY += allianceSign * 2.013;
        }
  
        if (alliance == Alliance.Blue)
       {
          isRed = false;
          // startPos = new Pose2d(centerX - startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
          startPos = new Pose2d(centerX - startLineOffset, centerY, new Rotation2d(Math.PI)); //startline the 2 causes the startup to be correct but we don't know why
        }
        else if (alliance == Alliance.Red)
        { 
          isRed = true;
          startPos = new Pose2d(centerX + startLineOffset, centerY, new Rotation2d(0)); //startline
        }
        else
        {
          return false;
          // SmartDashboard.putString("Alliance", "Null");
          // isRed = false;
          // startPos = new Pose2d(0, 0, new Rotation2d(0));
        }
        m_drivetrain.resetOdometry(startPos);
        m_localizer.resetPose(startPos);
        SmartDashboard.putNumber("RobotContainer/Start Pose X", startPos.getX());
        SmartDashboard.putNumber("RobotContainer/Start Pose Y", startPos.getY());
        SmartDashboard.putNumber("RobotContainer/Start Pose Rotation", startPos.getRotation().getRadians());
        SmartDashboard.putBoolean("Alliance", isRed); //True = Red, False = Blue ***NEED TO EDIT ON ELASTIC
        return true;
      }
      return false;
  }

  public boolean disabledPeriodic() 
  {
    return findStartPos();
  }

  @Override
  public void accept(String t) // gets called every time the selected position changes so the start position is reinitialized
  {
    haveInitStartPos = false;  
  }
}
