// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// TODO add get axis for both the left and right stick
// add them to tank drive left and right stick

package frc.robot;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.SystemMenuBar;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {
  WPI_TalonSRX _backLeftMotor = new WPI_TalonSRX(1);
  WPI_TalonSRX _frontLeftMotor = new WPI_TalonSRX(2);
  WPI_TalonSRX _frontRightMotor = new WPI_TalonSRX(3);
  WPI_TalonSRX _backRightMotor = new WPI_TalonSRX(4);

  boolean easy = true;
  double speed = .6;
  
  // Bundling the motors
  MotorControllerGroup leftMotors = new MotorControllerGroup(_frontLeftMotor, _backLeftMotor);
  MotorControllerGroup rightMotors = new MotorControllerGroup(_frontRightMotor, _backRightMotor);

  /* Construct drivetrain by providing master motor controllers */
  DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  // Operator
  private final Joystick m_stick = new Joystick(0);
  
  // Driver
  private final Joystick l_stick = new Joystick(1);
  private final Joystick r_stick = new Joystick(2);
  double lAdvance = l_stick.getY();
  double rAdvance = r_stick.getY();

  // Controller arcade drive axis
  double Advance = m_stick.getRawAxis(2);
  double zAdvance = m_stick.getRawAxis(3);
  double turn = m_stick.getRawAxis(1);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotors.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.


    // Speed adjust 
   if (m_stick.getRawButton(4)) { // FAAAASST
    speed = 1;
  } else if (m_stick.getRawButton(3)) { // Mid
    //speed = .5;
    speed = .6;
     lAdvance = l_stick.getY();
  rAdvance = r_stick.getY();
  } else if (m_stick.getRawButton(2)) { // Slow
    //speed = .33;
    speed = .5;
    double lAdvance = l_stick.getY();
  double rAdvance = r_stick.getY();

  // Take control
  } else if (m_stick.getRawButton(1)) {
    // Take control. You need to hold not making an boolean for this and your not the one suppose to be driving >:(
    lAdvance = m_stick.getRawAxis(1);
    rAdvance = m_stick.getRawAxis(3);

    // Custom speed adjustments.
  } else if (m_stick.getRawButton(8) && (speed <=.9 )){
    speed += .01;
  }else if (m_stick.getRawButton(7) && (speed >.01 )) {
      speed -= .01;
  }  else {
     lAdvance = l_stick.getY();
   rAdvance = r_stick.getY();
  }

  System.out.println("Speed: " + speed);
  robotDrive.tankDrive(lAdvance*speed, -rAdvance*speed);

/*  Baby mode switcher: Switches modes to one joystick or two joysticks
    if (m_stick.getRawButton(2)) {
      easy = true;
      System.out.println("Easy");

    } else if (m_stick.getRawButton(3)) {
      easy = false;
      System.out.println("Hard");
      Advance = m_stick.getRawAxis(2);
      turn = m_stick.getRawAxis(1);
    }

      //robotDrive.arcadeDrive(Advance * 0.6, -turn * 0.6);
    //robotDrive.arcadeDrive((m_stick.getRawAxis(2)) * 0.6, -(m_stick.getRawAxis(1)) * 0.6);

    if (easy == true ) {
      Advance = m_stick.getRawAxis(0);
      turn = m_stick.getRawAxis(1);
      robotDrive.arcadeDrive(Advance*0.6, -turn * 0.6);
      //robotDrive.arcadeDrive((m_stick.getRawAxis(0)) * 0.6, -(m_stick.getRawAxis(1)) * 0.6);
    } else { 
      Advance = m_stick.getRawAxis(2);
      turn = m_stick.getRawAxis(1);
      robotDrive.arcadeDrive(Advance*0.7, -turn * 0.6);
      //robotDrive.arcadeDrive((m_stick.getRawAxis(2)) * 0.6, -(m_stick.getRawAxis(1)) * 0.6);
    }
    //robotDrive.arcadeDrive((m_stick.getRawAxis(2)) * 0.6, -(m_stick.getRawAxis(1)) * 0.6);
    */
  }
}
