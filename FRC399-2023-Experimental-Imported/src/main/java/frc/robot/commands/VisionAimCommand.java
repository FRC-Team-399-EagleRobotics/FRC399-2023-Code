// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class VisionAimCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivetrainSubsystem m_tank;

  private Limelight limelight;
  //private final double distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
 public VisionAimCommand(DrivetrainSubsystem m_tank, Limelight limelight) {
    this.limelight = limelight;
    this.m_tank = m_tank;
    addRequirements(m_tank);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    limelight.setLight(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pX = 0.1, pY = 0.1;

    double x = limelight.getX() * pX;
    double y = limelight.getY() * pY;

    
    double leftOut = x-y;
    double rightOut = -x-y;
    
    m_tank.setTank(leftOut, rightOut);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLight(true);
    m_tank.setTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    return false;
  }
}