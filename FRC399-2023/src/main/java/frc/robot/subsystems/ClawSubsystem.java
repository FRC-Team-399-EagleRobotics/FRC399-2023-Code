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
import frc.robot.constants.Constants.Claw;

public class ClawSubsystem extends SubsystemBase {
  private TalonSRX clawMotor1, clawMotor2;
  private Timer m_timer;

    // Variables to store state of Claw 
    double iPwr = 0.0;


  /** Creates a new ExampleSubsystem. */
  public ClawSubsystem() {
    clawMotor1 = new TalonSRX(Claw.clawMotor1_ID);
    clawMotor2 = new TalonSRX(Claw.clawMotor2_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 public void grab() {
    setPwr(-.25);

    // Activate motors for a certain time then stop to not squish the object
    /*if (m_timer.get() < 1) {
    setPwr(.1);
    }*/
  }


  public void reverseGrab() {
    setPwr(1);
  }

  public void endGrab() {
    setPwr(0);
  }

  public void setPwr(double i) {
    iPwr = i;
    clawMotor1.set(ControlMode.PercentOutput, i);
    clawMotor2.set(ControlMode.PercentOutput, i);
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}