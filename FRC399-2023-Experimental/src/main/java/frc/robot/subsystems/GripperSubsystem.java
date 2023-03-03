// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
  // Template change anything if requir
  private TalonSRX GripperMotor1, GripperMotor2;
  private Solenoid GripperSolenoid1, GripperSolenoid2;

  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    GripperMotor1 = new TalonSRX(Gripper.GripperMotor1_ID);
    GripperMotor1 = new TalonSRX(Gripper.GripperMotor2_ID);

    GripperSolenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, Gripper.SoleniodCim1_ID);
    GripperSolenoid2 = new Solenoid(PneumaticsModuleType.CTREPCM, Gripper.SoleniodCim2_ID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

<<<<<<< HEAD:FRC399-2023-Experimental/src/main/java/frc/robot/subsystems/GripperSubsystem.java
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

=======
>>>>>>> parent of b7104c6 (Wrapped up template code):FRC399-2023/src/main/java/frc/robot/subsystems/GripperSubsystem.java
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
