// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Tankdrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivetrainSubsystem m_tank;
  //private final double distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
 public Tankdrive(DrivetrainSubsystem m_tank, double lPwr, double rPwr) {
    this.m_tank = m_tank;

    //this.distance = subsystem.getEncoderMeters() + distance;

    //Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Is this a good place to call it? Or should I put it in robotcontainer
    double stickL = RobotContainer.leftJoy.getRawAxis(1);
    double stickR = RobotContainer.rightJoy.getRawAxis(1);

    if(Math.abs(stickL) < 0.15) {
      stickL = 0;
    }

    if(Math.abs(stickR) < 0.15) {
      stickR = 0;
    }

    m_tank.setTank(stickR, stickL);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_tank.setTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}