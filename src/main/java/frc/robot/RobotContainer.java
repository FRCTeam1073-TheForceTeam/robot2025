// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import frc.robot.commands.ClimberClawTeleop;
import frc.robot.commands.ClimberLiftTeleop;
import frc.robot.commands.CoralElevatorTeleop;
import frc.robot.commands.CoralElevatorToHeight;
import frc.robot.commands.CoralEndeffectorTeleop;
import frc.robot.commands.EngageClaw;
import frc.robot.commands.LoadCoral;
import frc.robot.commands.ScoreCoral;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TroughScoreCoral;
import frc.robot.commands.ZeroClaw;
import frc.robot.commands.ZeroElevator;
import frc.robot.commands.ZeroLift;
import frc.robot.commands.Autos.AutoCenterStart;
import frc.robot.commands.Autos.AutoLeftStart;
import frc.robot.commands.Autos.AutoRightStart;
import frc.robot.commands.Autos.GenericL0;
import frc.robot.commands.Autos.RaiseLift;
import frc.robot.commands.Autos.ZeroClawAndLift;
import frc.robot.subsystems.AprilTagFinder;
import frc.robot.subsystems.ClimberClaw;
import frc.robot.subsystems.ClimberLift;
import frc.robot.subsystems.CoralElevator;
import frc.robot.subsystems.CoralEndeffector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.FieldMap;
import frc.robot.subsystems.Localizer;
import frc.robot.subsystems.MapDisplay;
import frc.robot.subsystems.OI;

public class RobotContainer 
{
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OI m_OI = new OI();
  private final AprilTagFinder m_aprilTagFinder = new AprilTagFinder();
  private final Field2d m_field = new Field2d();
  private final FieldMap m_fieldMap = new FieldMap();
  private final Localizer m_localizer = new Localizer(m_drivetrain, m_fieldMap, m_aprilTagFinder);
  private final MapDisplay m_MapDisplay = new MapDisplay(m_drivetrain, m_localizer, m_fieldMap);
  private final ClimberClaw m_climberClaw = new ClimberClaw();
  private final ClimberLift m_climberLift = new ClimberLift();
  private final ClimberClawTeleop m_climberClawTeleop = new ClimberClawTeleop(m_climberClaw, m_OI);
  private final ClimberLiftTeleop m_climberLiftTeleop = new ClimberLiftTeleop(m_climberLift, m_OI);
  private final CoralElevator m_coralElevator = new CoralElevator();
  private final CoralEndeffector m_coralEndeffector = new CoralEndeffector();
  private final ZeroElevator m_zeroElevator = new ZeroElevator(m_coralElevator, m_OI);
  private final CoralElevatorTeleop m_coralElevatorTeleop = new CoralElevatorTeleop(m_coralElevator, m_OI);
  private final CoralEndeffectorTeleop m_coralEndeffectorTeleop = new CoralEndeffectorTeleop(m_coralEndeffector, m_OI);
  private final LoadCoral m_loadCoral = new LoadCoral(m_coralEndeffector);
  private final ScoreCoral m_scoreCoral = new ScoreCoral(m_coralEndeffector);
  private final ZeroClaw m_zeroClaw = new ZeroClaw(m_climberClaw);
  private final ZeroLift m_zeroLift = new ZeroLift(m_climberLift);
  private final ZeroClawAndLift m_zeroClawAndLift = new ZeroClawAndLift();
  private final RaiseLift m_raiseLift = new RaiseLift(m_climberLift, m_OI);
  private final EngageClaw m_engageClaw = new EngageClaw(m_climberClaw);
  private final CoralElevatorToHeight m_coralElevatorToL2 = new CoralElevatorToHeight(m_coralElevator, m_OI, 2);
  private final CoralElevatorToHeight m_coralElevatorToL3 = new CoralElevatorToHeight(m_coralElevator, m_OI, 3);
  private final TroughScoreCoral m_troughScoreCoral = new TroughScoreCoral(m_coralEndeffector, m_coralElevator);
  private final CancelLoadCoral m_cancelLoadCoral = new CancelLoadCoral(m_coralEndeffector);
  private final AlignToTag m_alignToTag = new AlignToTag(m_drivetrain, m_localizer, m_fieldMap, m_OI);

  private final TeleopDrive m_teleopCommand = new TeleopDrive(m_drivetrain, m_OI, m_aprilTagFinder, m_localizer);

  private boolean isRed;
  private int level;

  private final SendableChooser<String> m_positionChooser = new SendableChooser<>();
  private static final String noPositionAuto = "No Position";
  private static final String rightAuto = "Right Auto";
  private static final String leftAuto = "Left Auto";
  private static final String centerAuto = "Center Auto";
  
  private final SendableChooser<String> m_levelChooser = new SendableChooser<>();
  private static final String testLevel = "Test Level";
  private static final String noLevelAuto = "No Level";
  private static final String level0 = "Level 0";
  private static final String level1 = "Level 1";
  private static final String level2 = "Level 2";
  private static final String level3 = "Level 3";
  private static final String level4 = "Level 4";
  private static final String zeroClawAndLift = "Zero Claw And Lift";

  public RobotContainer() 
  {
    CommandScheduler.getInstance().setDefaultCommand(m_drivetrain, m_teleopCommand);
    CommandScheduler.getInstance().setDefaultCommand(m_climberClaw, m_climberClawTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_climberLift, m_climberLiftTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_coralElevator, m_coralElevatorTeleop);
    CommandScheduler.getInstance().setDefaultCommand(m_coralEndeffector, m_coralEndeffectorTeleop);

    SmartDashboard.putData(m_drivetrain);
    SmartDashboard.putData(m_OI);
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData(m_localizer);

    m_positionChooser.setDefaultOption("No Position", noPositionAuto);
    m_positionChooser.addOption("Right Auto", rightAuto);
    m_positionChooser.addOption("Left Auto", leftAuto);
    m_positionChooser.addOption("Center Auto", centerAuto);
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


    configureBindings();
  }

  private void configureBindings() {
    Trigger raiseLift = new Trigger(m_OI::getOperatorAButton);
      raiseLift.onTrue(m_raiseLift);
    Trigger engageClaw = new Trigger(m_OI::getOperatorBButton);
      engageClaw.onTrue(m_engageClaw);
    Trigger zeroElevator = new Trigger(m_OI::getOperatorLeftJoystickPress);
      zeroElevator.onTrue(m_zeroElevator);
    Trigger loadCoral = new Trigger(m_OI::getOperatorXButton);
      loadCoral.onTrue(m_loadCoral);
    Trigger scoreCoral = new Trigger(m_OI::getOperatorYButton);
      scoreCoral.onTrue(m_scoreCoral);
    Trigger zeroClawAndLift = new Trigger(m_OI::getOperatorRightJoystickPress);
      zeroClawAndLift.onTrue(ZeroClawAndLift.create(m_climberClaw, m_climberLift));
    Trigger elevatorL2 = new Trigger(m_OI :: getOperatorDPadRight);
      elevatorL2.whileTrue(m_coralElevatorToL2);
    Trigger elevatorL3 = new Trigger(m_OI :: getOperatorDPadDown);
      elevatorL3.whileTrue(m_coralElevatorToL3);
    Trigger troughScore = new Trigger(m_OI::getOperatorDPadUp);
      troughScore.onTrue(m_troughScoreCoral);
    Trigger cancelLoadCoral = new Trigger(m_OI::getOperatorRightTrigger);
      cancelLoadCoral.onTrue(m_cancelLoadCoral);
    Trigger alignToTag = new Trigger(m_OI::getDriverAlignToTag);
      alignToTag.whileTrue(m_alignToTag);
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
      case zeroClawAndLift:
        return ZeroClawAndLift.create(m_climberClaw, m_climberLift);
      case noPositionAuto:
        return null;
      case leftAuto:
        return AutoLeftStart.create(level, isRed, m_drivetrain, m_localizer, m_climberClaw, m_climberLift);
      case rightAuto:
        return AutoRightStart.create(level, isRed, m_drivetrain, m_localizer, m_climberClaw, m_climberLift);
      case centerAuto:
        return AutoCenterStart.create(level, isRed, m_drivetrain, m_localizer, m_climberClaw, m_climberLift);
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
  }

  public boolean findStartPos() 
  {
      //create a bool for pose is set
      double centerY = 4.026;
      int allianceSign = 1;
      String selectedAuto = m_positionChooser.getSelected();
      
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
        if (selectedAuto.equals(leftAuto)) {
          centerY -= allianceSign * 2.013;
        }
        else if(selectedAuto.equals(rightAuto)) {
          centerY += allianceSign * 2.013;
        }
  
        if (alliance == Alliance.Blue)
        {
          isRed = false;
          startPos = new Pose2d(centerX - startLineOffset, centerY, new Rotation2d(Math.PI)); //startline
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
        SmartDashboard.putBoolean("isRed", isRed);
        return true;
      }
      return false;
  }

  public boolean disabledPeriodic() 
  {
    return findStartPos();
  }
}
