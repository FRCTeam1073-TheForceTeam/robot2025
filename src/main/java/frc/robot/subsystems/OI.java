// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase
{
  public enum BUTTONS{
    A(1),
    B(2),
    X(3),
    Y(4), 
    LeftJoystickY(1),
    LeftJoystickX(0),
    RightJoystickY(5),
    RightJoystickX(4),
    LeftJoystickPress(9),
    RightJoystickPress(10),
    //TODO Fix DPad
    // DPadUp(),
    // DPadLeft,
    // DPadDown,
    // DPadRight,
    LeftBumper(5),
    RightBumper(6), 
    LeftTrigger(2),
    RightTrigger(3),
    ViewButton(7),
    MenuButton(8);

    private int buttonValue;

    BUTTONS(int buttonValue){
      this.buttonValue = buttonValue;
    }
    public int getButtonVal(){
      return buttonValue;
    }
  }


  public enum PRIMARYPADBUTTONS {
    L1(5),
    L2(4),
    L3(3),
    L4(2),
    BargeScore(1),
    TopRed(6),
    MiddleRed(7),
    BottomRed(8);

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
    LoadAlgae(2),
    TwoPlayer(7),
    ScoreAlgae(4),
    LoadCoral(5),
    MiddleRed(6),
    ScoreCoral(7),
    HighAlgae(3),;


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

    public void zeroDriverController() 
  {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = getDriverLeftX();
    LEFT_Y_ZERO = getDriverLeftY();
    RIGHT_X_ZERO = getDriverRightX();
    RIGHT_Y_ZERO = getDriverRightY();
  }

  /** The following methods return quality-controlled values from the driver controller */
  public double getDriverLeftX() 
  {
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  public double getDriverLeftY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
  }

  public double getDriverRightX() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.RightJoystickX.getButtonVal()) - RIGHT_X_ZERO, -1, 1);
  }

  public double getDriverRightY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(BUTTONS.RightJoystickY.getButtonVal()) - RIGHT_Y_ZERO, -1, 1);
  }

  public double getDriverTranslateX()
  {
    return getDriverLeftX();
  }

  public double getDriverTranslateY()
  {
    return getDriverLeftY();
  }

  public double getDriverRotate()
  {
    return getDriverRightX();
  }

  public double getDriverRightTrigger()
  {
    return driverController.getRawAxis(BUTTONS.RightTrigger.getButtonVal());
  }

  public double getDriverLeftTrigger()
  {
    return driverController.getRawAxis(BUTTONS.LeftTrigger.getButtonVal());
  }

  public boolean getDriverLeftBumper(){
    return parkingBrakeDebouncer.calculate(driverController.getRawButton(BUTTONS.LeftBumper.getButtonVal()));
  }

  public boolean getDriverRightBumper(){
    return fieldCentricDebouncer.calculate(driverController.getRawButton(BUTTONS.RightBumper.getButtonVal()));
  }

  /** Returns a specified button from the driver controller */
  public boolean getDriverRawButton(int i) 
  {
    return driverController.getRawButton(i);
  }

  public boolean getDriverAButton(){
    return aDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.A.getButtonVal()));
  }

  public boolean getDriverBButton(){
    return bDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.B.getButtonVal()));
  }

  public boolean getDriverXButton()
  {
    return xDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.X.getButtonVal()));
  }

  public boolean getDriverYButton()
  {
    return yDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.Y.getButtonVal()));
  }

  public boolean getDriverMenuButton(){
    return menuDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.MenuButton.getButtonVal()));
  }

  public boolean getDriverViewButton(){
    return viewDriverButtonDebouncer.calculate(driverController.getRawButton(BUTTONS.ViewButton.getButtonVal()));
  }

  public boolean getDriverDPadUp()
  {
    return (driverController.getPOV() == 0);
  }

  public boolean getDriverDPadDown()
  {
    return (driverController.getPOV() == 180);
  }

  public boolean getDriverDPadLeft()
  {
    return (driverController.getPOV() == 270);
  }

  public boolean getDriverDPadRight()
  {
    return (driverController.getPOV() == 90);
  }

  //TODO William-What is this?????
  public boolean getDriverAlignButtons()
  {
    return getDriverAButton() || getDriverViewButton() || getDriverXButton() || getDriverYButton();
  }

  public boolean getDriverLeftJoystickPress(){
    return getDriverRawButton(BUTTONS.LeftJoystickPress.getButtonVal());
  }

  public void rumble() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 1);
  }

  public void stopRumble() {
    // OI.driverController.setRumble(RumbleType.kBothRumble, 0);
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
  public double getOperatorLeftX() {
    if(Math.abs(operatorSecondaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickX.getButtonVal())) < 0.1){
      return 0.0;
    }
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(operatorSecondaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickX.getButtonVal()) - LEFT_X_ZERO, -1, 1);
  }

  public double getOperatorLeftY() {
    if(Math.abs(operatorSecondaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickY.getButtonVal())) < 0.1){
      return 0.0;
    }
    return MathUtil.clamp(operatorSecondaryController.getRawAxis(SECONDARYPADBUTTONS.LeftJoystickY.getButtonVal()) - LEFT_Y_ZERO, -1, 1);
  }

  /** Returns a specified button from the operator controller */
  public boolean getOperatorPrimaryRawButton(int i) {
    return operatorPrimaryController.getRawButton(i);
  }

  public boolean getOperatorSecondaryRawButton(int i) {
    return operatorSecondaryController.getRawButton(i);
  }

  public boolean getOperatorL1() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L1.getButtonVal());
  }

  public boolean getOperatorL2() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L2.getButtonVal());
  }

  public boolean getOperatorL3() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L3.getButtonVal());
  }

  public boolean getOperatorL4() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.L4.getButtonVal());
  }

  public boolean getOperatorDisengageClimber() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.DisengageClimber.getButtonVal());
  }

  public boolean getOperatorZeroClimber() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ZeroClimber.getButtonVal());
  }

  public boolean getOperatorLoadCoral() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.LoadCoral.getButtonVal());
  }

  public boolean getOperatorScoralCoral() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ScoreCoral.getButtonVal());
  }

  public boolean getOperatorEngageClimber() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.EngageClimber.getButtonVal());
  }

  //TODO: haven't mapped these on controller
  public boolean getOperatorBargeScoreButton() {
    return getOperatorPrimaryRawButton(PRIMARYPADBUTTONS.BargeScore.getButtonVal());
  }

  public boolean getOperatorHighAlgae() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.HighAlgae.getButtonVal());
  }

  public boolean getOperatorLoadAlgae() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.LoadAlgae.getButtonVal());
  }

  public boolean getOperatorScoreAlgae() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ScoreAlgae.getButtonVal());
  }

  public boolean getOperatorTwoPlayerButton() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.TwoPlayer.getButtonVal());
  }

  public boolean getOperatorZeroElevator() {
    return getOperatorSecondaryRawButton(SECONDARYPADBUTTONS.ZeroElevator.getButtonVal());
  }

  public boolean getOperatorMiddleRedButton() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.MiddleRed.getButtonVal());
  }

  public boolean getOperatorTopRedButton() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.TopRed.getButtonVal());
  }

  public boolean getOperatorBottomRedButton() {
    return getOperatorSecondaryRawButton(PRIMARYPADBUTTONS.BottomRed.getButtonVal());
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