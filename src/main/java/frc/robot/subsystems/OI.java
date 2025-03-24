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

public class OI extends OldOI
{

  public enum PRIMARYPADBUTTONS {
    L1(1),
    L2(2),
    L3(3),
    L4(4),
    DisengageClimber(5),
    ZeroClimber(6),
    EngageClimber(7),
    BargeScore(8),
    AlgaeToggle(10),
    LoadAlgae(11),
    ScoreAlgae(12),
    LeftJoystickY(1),
    LeftJoystickX(0);

    private int buttonValue;

    PRIMARYPADBUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }

  public enum SECONDARYPADBUTTONS {
    IntakeAlgae(1),
    Destruct(2),
    ProccessorScore(3),
    ScoreCoral(7),
    LoadCoral(5),
    MiddleWhite(6);

    private int buttonValue;

    SECONDARYPADBUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }

  // Declares our controller variable
  public static Joystick driverController;
  public static Joystick operatorPrimaryController;
  public static Joystick operatorSecondaryController;

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
    if(Math.abs(operatorPrimaryController.getRawAxis(PRIMARYPADBUTTONS.LeftJoystickX.getButtonVal())) < 0.1){
      return 0.0;
    }
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(operatorPrimaryController.getRawAxis(PRIMARYPADBUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  @Override
  public double getOperatorLeftY() {
    if(Math.abs(operatorPrimaryController.getRawAxis(PRIMARYPADBUTTONS.LeftJoystickY.getButtonVal())) < 0.1){
      return 0.0;
    }
    return MathUtil.clamp(operatorPrimaryController.getRawAxis(PRIMARYPADBUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
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
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.DisengageClimber.getButtonVal());
  }

  @Override
  public boolean getOperatorBButton() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.ZeroClimber.getButtonVal());
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
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.EngageClimber.getButtonVal());
  }

  //TODO: haven't mapped these on controller
  @Override
  public boolean getOperatorBargeScoreButton() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.BargeScore.getButtonVal());
  }

  @Override
  public boolean getOperatorAlgaeToggle() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.AlgaeToggle.getButtonVal());
  }

  @Override
  public boolean getOperatorLoadAlgae() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.LoadAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorScoreAlgae() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.ScoreAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorFloorIntake() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.IntakeAlgae.getButtonVal());
  }

  @Override
  public boolean getOperatorProccessorScore() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ProccessorScore.getButtonVal());
  }

  @Override
  public boolean getDummyButton() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.Destruct.getButtonVal());
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