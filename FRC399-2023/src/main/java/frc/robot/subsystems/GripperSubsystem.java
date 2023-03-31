// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants.Arm;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.constants.Constants.Gripper;

public class GripperSubsystem extends SubsystemBase {
  private TalonSRX gripperMotor1, gripperMotor2;
  private Timer m_timer;

    // Variables to store state of Claw 
    double iPwr = 0.0;


  /** Creates a new ExampleSubsystem. */
  public GripperSubsystem() {
    gripperMotor1 = new TalonSRX(Gripper.gripperMotor1_ID);
    gripperMotor2 = new TalonSRX(Gripper.gripperMotor2_ID);

    gripperMotor1.setNeutralMode(NeutralMode.Brake);
    gripperMotor2.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 public void grip() {
    setPwr(-1);

    // Activate motors for a certain time then stop to not squish the object
    /*if (m_timer.get() < 1) {
    setPwr(.1);
    }*/
  }


  public void reverseGrip() {
    setPwr(1);
  }

  public void endGrip() {
    setPwr(0);
  }

  public void setPwr(double i) {
    iPwr = i;
    gripperMotor1.set(ControlMode.PercentOutput, i);
    gripperMotor2.set(ControlMode.PercentOutput, i);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}