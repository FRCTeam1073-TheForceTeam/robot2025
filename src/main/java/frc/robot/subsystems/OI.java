// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase
{
  // Declares our controller variable
  public static Joystick driverController;
  public static Joystick operatorController;

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
  private boolean manualCollectMode = true;

  /** Creates a new OI. */
  public OI() 
  {
    // Sets the driver controller to a new joystick object at port 0
    driverController = new Joystick(0);
    operatorController = new Joystick(1);
    zeroDriverController();
    zeroOperatorController();
  }

  /** This method will be called once per scheduler run */
  @Override
  public void periodic() 
  {    
    if(getOperatorMenuButton()){
      setCollectMode(!manualCollectMode);
    }
    // You can add more smartdashboard printouts here for additional joysticks or buttons
  }

  public boolean getCollectMode(){
    return manualCollectMode;
  }

  public void setCollectMode(boolean collect){
    manualCollectMode = collect;
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
    return MathUtil.clamp(driverController.getRawAxis(0) - LEFT_X_ZERO, -1, 1);
  }

  public double getDriverLeftY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(1) - LEFT_Y_ZERO, -1, 1);
  }

  public double getDriverRightX() 
  {
    return MathUtil.clamp(driverController.getRawAxis(4) - RIGHT_X_ZERO, -1, 1);
  }

  public double getDriverRightY() 
  {
    return MathUtil.clamp(driverController.getRawAxis(5) - RIGHT_Y_ZERO, -1, 1);
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

  /** Returns a specified button from the driver controller */
  public boolean getDriverRawButton(int i) 
  {
    return driverController.getRawButton(i);
  }

  public double getDriverRightTrigger()
  {
    return driverController.getRawAxis(3);
  }

  public double getDriverLeftTrigger()
  {
    return driverController.getRawAxis(2);
  }

  // public boolean getFieldCentricToggle()
  // {
  //   return fieldCentricDebouncer.calculate(driverController.getRawButton(7));
  // }

  public boolean getDriverLeftBumper(){
    return parkingBrakeDebouncer.calculate(driverController.getRawButton(5));
  }

  public boolean getDriverRightBumper(){
    return fieldCentricDebouncer.calculate(driverController.getRawButton(6));
  }

  public boolean getDriverMenuButton(){
    return menuDriverButtonDebouncer.calculate(driverController.getRawButton(8));
  }

  public boolean getDriverAButton(){
    return aDriverButtonDebouncer.calculate(driverController.getRawButton(1));
  }

  public boolean getDriverViewButton(){
    return viewDriverButtonDebouncer.calculate(driverController.getRawButton(7));
  }

  public boolean getDriverBButton(){
    return bDriverButtonDebouncer.calculate(driverController.getRawButton(2));
  }

  public boolean getDriverYButton()
  {
    
    return yDriverButtonDebouncer.calculate(driverController.getRawButton(4));
  }

  public boolean getDriverXButton()
  {
    return xDriverButtonDebouncer.calculate(driverController.getRawButton(3));
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

  public boolean getDriverAlignButtons()
  {
    return getDriverAButton() || getDriverViewButton() || getDriverXButton() || getDriverYButton();
  }

  public boolean getDriverLeftJoystickPress(){
    return getDriverRawButton(9);
  }

  public void zeroOperatorController() {
    //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
    LEFT_X_ZERO = 0;
    LEFT_Y_ZERO = 0;
    RIGHT_X_ZERO = 0;
    RIGHT_Y_ZERO = 0;
    LEFT_X_ZERO = getOperatorLeftX();
    LEFT_Y_ZERO = getOperatorLeftY();
    RIGHT_X_ZERO = getOperatorRightX();
    RIGHT_Y_ZERO = getOperatorRightY();
  }

  /** The following methods return quality-controlled values from the operator controller */
  public double getOperatorLeftX() {
    if(Math.abs(operatorController.getRawAxis(0)) < 0.1){
      return 0.0;
    }
    // "Clamping" the value makes sure that it's still between 1 and -1 even if we have added an offset to it
    return MathUtil.clamp(operatorController.getRawAxis(0) - LEFT_X_ZERO, -1, 1);
  }

  public double getOperatorLeftY() {
    if(Math.abs(operatorController.getRawAxis(1)) < 0.1){
      return 0.0;
    }
    return -1.0 * MathUtil.clamp(operatorController.getRawAxis(1) - LEFT_Y_ZERO, -1, 1);
  }

  public double getOperatorRightX() {
    if(Math.abs(operatorController.getRawAxis(4)) < 0.1){
      return 0.0;
    }
    return MathUtil.clamp(operatorController.getRawAxis(4) - RIGHT_X_ZERO, -1, 1);
  }

  public double getOperatorRightY() {
    if(Math.abs(operatorController.getRawAxis(5)) < 0.1){
      return 0.0;
    }
    return -1.0 * MathUtil.clamp(operatorController.getRawAxis(5) - RIGHT_Y_ZERO, -1, 1);
  }

  /** Returns a specified button from the operator controller */
  public boolean getOperatorRawButton(int i) {
    return operatorController.getRawButton(i);
  }

  public boolean getOperatorAButton(){
    return getOperatorRawButton(1);
  }

  public boolean getOperatorXButton(){
    return getOperatorRawButton(3);
  }

  public boolean getOperatorYButton(){
    return getOperatorRawButton(4);
  }

  public boolean getOperatorBButton(){
    return getOperatorRawButton(2);
  }
/* 
  public double getOperatorRightTrigger(){
    return MathUtil.clamp(operatorController.getRawAxis(3), 0, 1);
  }*/

  public double getOperatorRightTrigger(){
    return MathUtil.clamp( operatorController.getRawAxis(3), 0, 1);
  }

  public double getOperatorLeftTrigger(){
    return MathUtil.clamp( operatorController.getRawAxis(2), 0, 1);
  }

  public boolean getOperatorViewButton() {
    return getOperatorRawButton(7);
  }

  public boolean getOperatorMenuButton() {
    return menuOperatorButtonDebouncer.calculate(operatorController.getRawButton(8));

  }

  public boolean getOperatorRightBumper(){
    return getOperatorRawButton(6);
  }

  public boolean getOperatorLeftBumper(){
    return getOperatorRawButton(5);
  }

  public boolean getOperatorDPadUp(){
    return (operatorController.getPOV() == 0);
  }

  public boolean getOperatorDPadDown(){
    return (operatorController.getPOV() == 180);
  }

  public boolean getOperatorDPadLeft(){
    return (operatorController.getPOV() == 270);
  }

  public boolean getOperatorDPadRight(){
    return (operatorController.getPOV() == 90);
  }

  public boolean getOperatorLeftJoystickPress(){
    return getOperatorRawButton(9);
  }

  public boolean getOperatorRightJoystickPress(){
    return getOperatorRawButton(10);
  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("OI");
    builder.addDoubleProperty("Driver Right Y", this::getDriverRightY, null);
    builder.addDoubleProperty("Driver Right X", this::getDriverRightX, null);
    builder.addDoubleProperty("Driver Left Y", this::getDriverLeftY, null);
    builder.addDoubleProperty("Driver Left X", this::getDriverLeftX, null);
    builder.addDoubleProperty("Operator Right Y", this::getOperatorRightY, null);
    builder.addDoubleProperty("Operator Right X", this::getOperatorRightX, null);
    builder.addDoubleProperty("Operator Left Y", this::getOperatorLeftY, null);
    builder.addDoubleProperty("Operator Left X", this::getOperatorLeftX, null);
    builder.addBooleanProperty("Manual Collect Mode", this::getCollectMode, null);

  }
}