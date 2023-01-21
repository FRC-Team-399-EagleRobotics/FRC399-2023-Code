// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberSubsystem extends SubsystemBase {
  // Calling out motors
  private WPI_TalonSRX climberL, climberR;
  private MotorControllerGroup climbMotors;
  private Solenoid climberSolenoid;

  double cPWR = 0;
  boolean cPos = false;
  
  public ClimberSubsystem() {
    // Calling motors
    climberL = new WPI_TalonSRX(Constants.Climber.leftClimberCim1_ID);
    climberR = new WPI_TalonSRX(Constants.Climber.rightClimberCim1_ID);
    climbMotors = new MotorControllerGroup(climberL, climberR);
    climberL.setNeutralMode(NeutralMode.Brake);
    climberR.setNeutralMode(NeutralMode.Brake);
    climberSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Climber.climberSolenoid_ID);
  }

  public void climberControl(double C)
  {
    cPWR = C;
    climbMotors.set(C);
  }

  public void setPos(boolean p) {
    cPos = p;
    climberSolenoid.set(p);
  }

  public void up(){
    climbMotors.set(-1);
  }

  public void down(){
    climbMotors.set(1);
  }

  public void active(){
    climberSolenoid.set(true);
  }

  public void inactive(){
    climberSolenoid.set(false);
  }

  public void endMotors(){
    climbMotors.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}