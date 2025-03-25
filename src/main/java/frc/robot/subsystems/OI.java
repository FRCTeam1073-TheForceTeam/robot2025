// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.PrimitiveIterator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// Free drive controller buttons: press left joystick, press right joystick
// Free operator controller buttons: Right joystick y, left joystick x, right joystick x
import frc.robot.commands.DisengageClimber;
import frc.robot.commands.EngageClimber;
import frc.robot.commands.ZeroClimber;
import frc.robot.commands.ZeroElevator;

public class OI extends OldOI
{

  public enum PRIMARYPADBUTTONS {
    L1(5),
    L2(4),
    L3(3),
    L4(2),
    BargeScore(1),
    IntakeAlgae(6),
    Destruct(7),
    ProccessorScore(8);

    private int buttonValue;

    PRIMARYPADBUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }

  public enum SECONDARYPADBUTTONS {
    DisengageClimber(8),
    ZeroClimber(9),
    EngageClimber(10),
    LeftJoystickY(1),
    LeftJoystickX(0),
    ZeroElevator(1),
    LoadCoral(5),
    MiddleWhite(6),
    ScoreCoral(7),
    AlgaeToggle(3),
    LoadAlgae(2),
    ScoreAlgae(4);


    private int buttonValue;

    SECONDARYPADBUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }

  // Declares our controller variable
  private Joystick driverController;
  private Joystick operatorPrimaryController;
  private Joystick operatorSecondaryController;

  public Debouncer fieldCentricDebouncer = new Debouncer(0.05);
  public Debouncer parkingBrakeDebouncer = new Debouncer(0.05);
  public Debouncer menuDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer aDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer bDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer yDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer xDriverButtonDebouncer = new Debouncer(0.05);
  public Debouncer menuOperatorButtonDebouncer = new Debouncer(0.13);
  public Debouncer viewDriverButtonDebouncer = new Debouncer(0.05);

  // Declares the "zero" value variables (which allow us to compensate for joysticks that are a little off)
  private double LEFT_X_ZERO;
  private double LEFT_Y_ZERO;
  private double RIGHT_X_ZERO;
  private double RIGHT_Y_ZERO;

  /** Creates a new OI. */
  public OI() 
  {
    // Sets the driver controller to a new joystick object at port 0
    driverController = new Joystick(0);
    operatorPrimaryController = new Joystick(1);
    operatorSecondaryController = new Joystick(2);
    zeroDriverController();
    zeroOperatorControllers();
  }

  public void zeroOperatorControllers() 
  {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    LEFT_X_ZERO = getOperatorLeftX();
    LEFT_Y_ZERO = getOperatorLeftY();
  }

  /** The following methods return quality-controlled values from the operator controller */
  @Override
  public double getOperatorLeftX() {
    if(Math.abs(operatorPrimaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickX.getButtonVal())) < 0.1){
      return 0.0;
    }
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(operatorPrimaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  @Override
  public double getOperatorLeftY() {
    if(Math.abs(operatorPrimaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickY.getButtonVal())) < 0.1){
      return 0.0;
    }
    return MathUtil.clamp(operatorPrimaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
  }

  /** Returns a specified button from the operator controller */
  public boolean getOperatorPrimaryRawButton(int i) {
    return operatorPrimaryController.getRawButton(i);
  }

  public boolean getOperatorSecondaryRawButton(int i) {
    return operatorSecondaryController.getRawButton(i);
  }

  @Override
  public boolean getOperatorDPadUp() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L1.getButtonVal());
  }

  @Override
  public boolean getOperatorDPadRight() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L2.getButtonVal());
  }

  @Override
  public boolean getOperatorDPadDown() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L3.getButtonVal());
  }

  @Override
  public boolean getOperatorDPadLeft() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L4.getButtonVal());
  }

  @Override
  public boolean getOperatorAButton() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.DisengageClimber.getButtonVal());
  }

  @Override
  public boolean getOperatorBButton() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.ZeroClimber.getButtonVal());
  }

  @Override
  public boolean getOperatorXButton() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.LoadCoral.getButtonVal());
  }

  @Override
  public boolean getOperatorYButton() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ScoreCoral.getButtonVal());
  }

  @Override
  public boolean getOperatorMenuButton() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.EngageClimber.getButtonVal());
  }

  //TODO: haven't mapped these on controller
  @Override
  public boolean getOperatorBargeScoreButton() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.BargeScore.getButtonVal());
  }

  @Override
  public boolean getOperatorAlgaeToggle() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.AlgaeToggle.getButtonVal());
  }

  @Override
  public boolean getOperatorLoadAlgae() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.LoadAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorScoreAlgae() {
    return getOperatorPrimaryRawButton(SECONDARYPADBUTTONS.ScoreAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorFloorIntake() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.IntakeAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorProccessorScore() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.ProccessorScore.getButtonVal());
  }

  @Override
  public boolean getDummyButton() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.Destruct.getButtonVal());
  }

  @Override
  public boolean getOperatorLeftJoystickPress() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ZeroElevator.getButtonVal());
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("OI");
    builder.addDoubleProperty("Driver Right Y", this::getDriverRightY, null);
    builder.addDoubleProperty("Driver Right X", this::getDriverRightX, null);
    builder.addDoubleProperty("Driver Left Y", this::getDriverLeftY, null);
    builder.addDoubleProperty("Driver Left X", this::getDriverLeftX, null);
    builder.addDoubleProperty("Operator Left Y", this::getOperatorLeftY, null);
    builder.addDoubleProperty("Operator Left X", this::getOperatorLeftX, null);
  }
}