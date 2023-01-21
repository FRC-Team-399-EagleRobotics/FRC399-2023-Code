// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private Solenoid hoodSolenoid;
  private Solenoid shooterLed;
  private TalonFX shooterL, shooterR;
  private Timer m_timer;

  //Motor Setup -- Keep this hopefully doesn't bother anybody
  //BangBangController BANGshooter = new BangBangController();
  //SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(10, 20);

  // Variables to store shooter velocity and position
  double vel = 0.0;
  boolean pos = false;

  /**
   * Constructor
   */
  public ShooterSubsystem()
  {
    //Solenoid
    hoodSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Shooter.hoodSolenoid_ID);
    shooterLed = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

    // Motors 
    shooterL = new TalonFX(Constants.Shooter.shooterL_ID);
    shooterR = new TalonFX(Constants.Shooter.shooterR_ID);
    shooterL.setNeutralMode(NeutralMode.Coast);
    shooterR.setNeutralMode(NeutralMode.Coast);
    //shooterL.configOpenloopRamp(0.15);
    //shooterR.configOpenloopRamp(0.15);
    shooterL.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    shooterL.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));
    
    shooterR.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 45, 50, 1.0));
    shooterR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));
  }

  public void lowShot() {
    setVel(0.3);
    setHood(false);
  }
double error = 0;
double actualVel = 0;

double control = 0.0;

  public void highShot() {
    //setVel(0.68);

    double command = 13100;

    double ff = command / 19700;

    //ff *= 0.85;//0.78;

    actualVel = shooterL.getSelectedSensorVelocity();
    
    error = command - actualVel;

    if(actualVel < command) {
      control = 1.0;
    } else if(actualVel >= command) {
      control = ff;
    }

    setVel(control);

    SmartDashboard.putNumber("Vel", actualVel);
    //System.out.println(actualVel);
    setHood(true);
  }

  public void funnyShot() {
    //setVel(0.65);

    double command = 12200;

    double ff = command / 19700;
    
    //ff *= 0.95;//0.78;

    actualVel = shooterL.getSelectedSensorVelocity();
    
    error = command - actualVel;
    
    actualVel = shooterL.getSelectedSensorVelocity();
        
    control = 0.0;
    
    if(actualVel < command) {
      control = 1;
    } else if(actualVel >= command) {
      control = ff;
    }
    
    setVel(control);

    setHood(false);
  }

  public void endShooter() {
    setVel(0);
    setHood(false);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run

    shooterLed.set(Math.abs(error) < 500 && Math.abs(control) > 0.001 );
  }

  // Basically Pwr for the shooters
  public void setVel(double v)
  {

    
    vel = v;
    shooterL.set(ControlMode.PercentOutput, v);
    shooterR.set(ControlMode.PercentOutput, -v);
  }

  public void setHood(boolean p)
  {
    pos = p;
    hoodSolenoid.set(p);
  }
  

}

