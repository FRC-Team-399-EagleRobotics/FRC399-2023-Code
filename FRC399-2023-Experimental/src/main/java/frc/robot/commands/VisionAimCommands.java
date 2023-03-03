// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.constants.swerveConstants.OIConstants;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.Limelight;

/** An example command that uses an example subsystem. */
public class VisionAimCommands extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem m_swerve;

  private Limelight limelight;
  //private final double distance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
 public VisionAimCommands(DriveSubsystem m_swerve, Limelight limelight) {
    this.limelight = limelight;
    this.m_swerve = m_swerve;
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    limelight.setLight(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pX = 0.02, pY = 0.04;

    double x = limelight.getX() * pX;
    double y = limelight.getY() * pY;


    double leftOut = x-y;
    double rightOut = -x-y;

            m_swerve.drive(
          -MathUtil.applyDeadband(leftOut, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(rightOut, OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(RobotContainer.m_driver.getRawAxis(2), OIConstants.kDriveDeadband),
          true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLight(true);
    m_swerve.setTank(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    
    return false;
  }
}