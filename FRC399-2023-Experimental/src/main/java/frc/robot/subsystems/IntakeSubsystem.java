// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  
  private Solenoid intakeSolenoid;
  private TalonSRX intakeMotor;
  private Timer m_timer;

  // Variables to store state of intake 
  double iPwr = 0.0;
  boolean iPos = false;

  /**
   * Constructor
   */
  public IntakeSubsystem() {
    intakeMotor = new TalonSRX(Constants.Intake.intakeMotor_ID);
    intakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Intake.intakeSolenoid_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: write iPwr and iPos to motor controller and solenoid outputs
  }

  public void extend() {
    setPos(true);
  }

  public void retract() {
    setPos(false);
  }

  public void intake() {
    setPwr(-1);
    setPos(true);
  }

  public void outTake() {
    setPwr(1);
    setPos(true);
  }

  public void endIntake() {
    setPwr(0);
    setPos(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPwr(double i) {
    iPwr = i;
    intakeMotor.set(ControlMode.PercentOutput, i);
  }

  public void setPos(boolean p) {
    iPos = p;
    intakeSolenoid.set(p);
  }

  public void autoIntake() {
    intake();
  }

}