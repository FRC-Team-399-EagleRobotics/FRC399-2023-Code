// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ConveyorSubsystem extends SubsystemBase {
  // TODO: instantiate conveyor motors
  private TalonSRX topConveyor, bottomConveyor;
  private Solenoid leftRed;
  private Solenoid leftBlue;
  private Solenoid rightRed;
  private Solenoid rightBlue;
  

  private AnalogInput leftSensor;
  private AnalogInput rightSensor;
  // Variables for motor power
  double aPwr = 0.0;
  double bPwr = 0.0;

  /**
   * Constructor.
   */
  public ConveyorSubsystem() {
    // Initialize conveyor motors
    topConveyor = new TalonSRX(Constants.Conveyor.topConveyor_ID);
    bottomConveyor = new TalonSRX(Constants.Conveyor.bottomConveyor_ID);

    leftRed = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
    leftBlue = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    rightRed = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    rightBlue = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

    leftSensor = new AnalogInput(0);
    rightSensor = new AnalogInput(1);

    topConveyor.setNeutralMode(NeutralMode.Brake);
    bottomConveyor.setNeutralMode(NeutralMode.Brake);

  }

  public void setConveyor(double a, double b) {
    topConveyor.set(ControlMode.PercentOutput, a);
    bottomConveyor.set(ControlMode.PercentOutput, b);
  }
  /**
   * Sets both conveyor motors
   */
  // Test speeds or time
  public void intake(){
    setPwr(0.25, -0.25);
  }
  public void store() {
    setPwr(0.8, -0.8);
  }

  public void spit(){
    setPwr(1, 1);
  }

  public void load() {
    setPwr(-1, -1);
  }

  public void endConveyor() {
    setPwr(0,0);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run

    double left = leftSensor.getAverageVoltage();
    double right = rightSensor.getAverageVoltage();

    SmartDashboard.putNumber("Left Sensor", left);
    SmartDashboard.putNumber("Right Sensor", right);

    boolean leftOut = left < 4;
    boolean rightOut = right < 4;

    leftRed.set(leftOut);
    leftBlue.set(leftOut);
    rightRed.set(rightOut);
    rightBlue.set(rightOut);
  }

  public void setPwr(double a, double b) {
    aPwr = a;
    bPwr = b;

    topConveyor.set(ControlMode.PercentOutput, a);
    bottomConveyor.set(ControlMode.PercentOutput, b);
  }
  
}
