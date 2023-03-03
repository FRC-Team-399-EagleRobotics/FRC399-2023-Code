// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
  private TalonSRX gripperMotor1, gripperMotor2;
  private Solenoid wristSolenoid, gripperSolenoid;
  private Timer m_timer;

    // Variables to store state of gripper 
    double iPwr = 0.0;
    boolean iPos = false;


  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    gripperMotor1 = new TalonSRX(Gripper.gripperMotor1_ID);
    gripperMotor2 = new TalonSRX(Gripper.gripperMotor2_ID);

    wristSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Gripper.wristSolenoid_ID);
    gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Gripper.gripperSolenoid_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 public void grab(double i) {
  iPwr = i;
  gripperMotor1.set(ControlMode.PercentOutput, -i);
  gripperMotor2.set(ControlMode.PercentOutput, i);
    setPos(true);
  }

  public void endGrab(double i) {
    iPwr = i;
    gripperMotor1.set(ControlMode.PercentOutput, i);
    gripperMotor2.set(ControlMode.PercentOutput, -i);
    setPos(false);
  }

  public void setPos(boolean p) {
    iPos = p;
    wristSolenoid.set(p);
    gripperSolenoid.set(p);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
